import os
import re
from pathlib import Path

from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate

SHAPES = ("circle", "square", "triangle", "star")
COMMANDS = ("start", "stop", "none")

# LLM이 실패/느릴 때를 위한 규칙 기반 1차 매칭
_SHAPE_ALIASES = {
    "circle":   ["circle", "원", "동그라미", "원형", "서클"],
    "square":   ["square", "사각형", "네모", "정사각형", "스퀘어"],
    "triangle": ["triangle", "삼각형", "세모", "트라이앵글"],
    "star":     ["star", "별", "별표", "별모양", "스타"],
}
_STOP_WORDS = ["정지", "멈춰", "멈춤", "중지", "스톱", "stop"]

ENV_PATH = str(Path(__file__).resolve().parent.parent / "resource" / ".env")


def rule_match(text: str):
    """LLM 없이 문자열만으로 도형/명령 추출. 실패 시 (None, None)."""
    shape = None
    for name, aliases in _SHAPE_ALIASES.items():
        if any(a in text for a in aliases):
            shape = name
            break
    command = "stop" if any(w in text for w in _STOP_WORDS) else (
        "start" if shape else "none"
    )
    return shape, command


class ShapeExtractor:
    def __init__(self):
        load_dotenv(dotenv_path=ENV_PATH)
        self.llm = ChatOpenAI(
            model="gpt-4o-mini",
            temperature=0.0,
            openai_api_key=os.getenv("OPENAI_API_KEY"),
        )
        prompt_content = """
당신은 사용자의 한국어 문장에서 조립할 '도형'과 '명령'을 추출합니다.

<도형 리스트>
circle, square, triangle, star

<명령 리스트>
start (조립/집기/시작 요청), stop (정지/중단 요청), none (해당 없음)

<출력 형식>
- 반드시 다음 한 줄 형식만 출력: shape / command
- 도형이 없으면 shape 자리에 none
- 다른 설명, 따옴표, 대괄호 절대 금지

<매핑 규칙>
- 원, 동그라미, 원형 -> circle
- 네모, 사각형, 정사각형 -> square
- 세모, 삼각형 -> triangle
- 별, 별표, 별모양 -> star

<예시>
입력: "삼각형 조립해줘"        출력: triangle / start
입력: "동그라미 집어"          출력: circle / start
입력: "별 모양 끼워줘"         출력: star / start
입력: "그만 멈춰"              출력: none / stop
입력: "오늘 날씨 어때"         출력: none / none

<사용자 입력>
"{user_input}"
"""
        self.chain = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        ) | self.llm

    def extract(self, text: str):
        """returns (shape or None, command in COMMANDS)"""
        shape, command = rule_match(text)
        if shape or command == "stop":
            return shape, command

        raw = self.chain.invoke({"user_input": text}).content
        parts = [p.strip().lower() for p in re.split(r"[/\n]", raw) if p.strip()]
        shape = parts[0] if parts and parts[0] in SHAPES else None
        command = parts[1] if len(parts) > 1 and parts[1] in COMMANDS else (
            "start" if shape else "none"
        )
        return shape, command