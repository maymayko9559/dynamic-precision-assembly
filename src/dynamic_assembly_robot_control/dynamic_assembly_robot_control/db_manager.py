# db_manager.py
import psycopg2

class DbManager:
    def __init__(self, host="localhost", port="5432", database="assembly_db", user="rokey", password="rokey_password"):
        try:
            # PostgreSQL 서버 연결
            self.conn = psycopg2.connect(
                host=host,
                port=port,
                database=database,
                user=user,
                password=password
            )
            self.cursor = self.conn.cursor()
            print("PostgreSQL 데이터베이스 연결 성공!")
            self.create_table()
        except Exception as e:
            print(f"PostgreSQL 연결 실패: {e}")
            self.conn = None

    def create_table(self):
        """실무형 비전 로그 테이블 생성"""
        if not self.conn:
            return
        
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS vision_logs (
                id SERIAL PRIMARY KEY,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                detect_type VARCHAR(20),
                shape VARCHAR(50),
                x REAL,
                y REAL,
                z REAL,
                angle REAL
            )
        ''')
        self.conn.commit()

    def insert_data(self, detect_type, shape, x, y, z, angle):
        """데이터 적재 (실무에서는 쿼리 인젝션 방지를 위해 파라미터 바인딩 사용)"""
    


        if not self.conn:
            return

        try:
            query = '''
                INSERT INTO vision_logs (detect_type, shape, x, y, z, angle)
                VALUES (%s, %s, %s, %s, %s, %s)
            '''
            self.cursor.execute(query, (detect_type, shape, x, y, z, angle))
            print("좌표값 받아옴")
            self.conn.commit()
        except Exception as e:
            print(f"데이터 삽입 중 에러 발생: {e}")
            self.conn.rollback()  # 에러 발생 시 롤백

    def close(self):
        """연결 종료"""
        if self.cursor:
            self.cursor.close()
        if self.conn:
            self.conn.close()
            print("PostgreSQL 연결이 안전하게 종료되었습니다.")


if __name__ == "__main__":
    db = DbManager()