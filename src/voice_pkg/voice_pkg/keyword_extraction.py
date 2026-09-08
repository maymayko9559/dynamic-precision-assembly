import os
from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
import warnings
from langchain_core.prompts import PromptTemplate
from pathlib import Path

ENV_PATH = str(Path(__file__).resolve().parent.parent / "resource" / ".env")


class ExtractKeyword:
    def __init__(self):
        load_dotenv(dotenv_path=ENV_PATH)
        openai_api_key = os.getenv("OPENAI_API_KEY")
        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )
        
        prompt_content = """
            당신은 사용자의 문장에서 특정 도구와 목적지를 추출해야 합니다.

            <목표>
            - 문장에서 다음 리스트에 포함된 도구를 최대한 정확히 추출하세요.
            - 문장에 등장하는 도구의 목적지(어디로 옮기라고 했는지)도 함께 추출하세요.

            <도구 리스트>
            - hammer, screwdriver, wrench, pos1, pos2, pos3

            <출력 형식>
            - 다음 형식을 반드시 따르세요: [도구1 도구2 ... / pos1 pos2 ...]
            - 도구와 위치는 각각 공백으로 구분
            - 도구가 없으면 앞쪽은 공백 없이 비우고, 목적지가 없으면 '/' 뒤는 공백 없이 비웁니다.
            - 도구와 목적지의 순서는 등장 순서를 따릅니다.

            <특수 규칙>
            - 명확한 도구 명칭이 없지만 문맥상 유추 가능한 경우(예: "못 박는 것" → hammer)는 리스트 내 항목으로 최대한 추론해 반환하세요.
            - 다수의 도구와 목적지가 동시에 등장할 경우 각각에 대해 정확히 매칭하여 순서대로 출력하세요.

            <예시>
            - 입력: "hammer를 pos1에 가져다 놔"  
            출력: hammer / pos1

            - 입력: "왼쪽에 있는 해머와 wrench를 pos1에 넣어줘"  
            출력: hammer wrench / pos1

            - 입력: "왼쪽에 있는 hammer를 줘"  
            출력: hammer /

            - 입력: "왼쪽에 있는 못박을 수 있는 것을 줘"  
            출력: hammer /

            - 입력: "hammer는 pos2에 두고 screwdriver는 pos1에 둬"  
            출력: hammer screwdriver / pos2 pos1

            <사용자 입력>
            "{user_input}"                
        """
        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm

    def extract_keyword(self, output_message):
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response.content.strip().split("/")
        if len(result) != 2:
            warnings.warn("The object list is more than one.")
            return None

        object, destination = result[0], result[1]
        object = object.split()

        destination = destination.split()

        print(f"llm's response(object): {object}")
        print(f"llm's response(destination): {destination}")

        return object, destination


if __name__ == "__main__":
    output_message = ""
    extract_keyword = ExtractKeyword()
    keyword = extract_keyword.extract_keyword(output_message)
