import os
import json

from openai import OpenAI

from dotenv import load_dotenv

from ament_index_python.packages import get_package_share_directory


load_dotenv(
    dotenv_path=os.path.join(
        get_package_share_directory("voice_pkg"),
        "resource",
        ".env"
    )
)

openai_api_key = os.getenv("OPENAI_API_KEY")

SHAPES = ["circle", "square", "triangle", "star"]

MODEL_NAME = "text-embedding-ada-002"


def main(args=None):

    client = OpenAI(api_key=openai_api_key)

    embeddings = {}

    for shape in SHAPES:

        response = client.embeddings.create(
            model=MODEL_NAME,
            input=shape
        )

        embeddings[shape] = response.data[0].embedding

        print(f"{shape}: {len(embeddings[shape])}")

    with open("shape_embedding.json", "w") as f:
        json.dump(embeddings, f)

    print("shape_embedding.json 생성 완료")


if __name__ == "__main__":
    main()