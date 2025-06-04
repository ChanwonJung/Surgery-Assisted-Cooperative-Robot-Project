# ros2 service call /get_keyword std_srvs/srv/Trigger "{}"

import os
import rclpy
import pyaudio
import json
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from langchain.chat_models import ChatOpenAI
from langchain.prompts import PromptTemplate
from langchain.chains import LLMChain

from std_srvs.srv import Trigger
from voice_processing.MicController import MicController, MicConfig

from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT

############ Package Path & Environment Setting ############
current_dir = os.getcwd()
package_path = get_package_share_directory("pick_and_place_voice")

is_laod = load_dotenv(dotenv_path=os.path.join(f"{package_path}/resource/.env"))
openai_api_key = os.getenv("OPENAI_API_KEY")

############ AI Processor ############
# class AIProcessor:
#     def __init__(self):



############ GetKeyword Node ############
class GetKeyword(Node):
    def __init__(self):
        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )
        self.tool_dict = self.load_tool_list()
        prompt_content = prompt_content = """
            당신은 사용자의 문장에서 특정 도구를 추출해야 합니다.
            
            <목표>
            - 문장에서 다음 리스트에 포함된 도구를 최대한 정확히 추출하세요.
            - 문장에 등장하는 도구를 영어 또는 한국어로 추출하세요.
            - 출력은 반드시 단일 문자열이어야 합니다.
            
            <도구 리스트>
            - Scalpel, Yankauer, Hemostat, Mayo_metz,Forceps, wound, Anesthesia

            <출력 형식>
            - 도구가 하나일 경우: "Scalpel"
            - 도구가 여러 개일 경우: "Scalpel Yankauer"
            - 도구가 없을 경우: ""
            - 도구와 도구 사이는 공백 하나로만 구분
            - 대괄호([]), 따옴표('), 쉼표(,) 등 특수문자 사용 금지
            - 도구의 순서는 등장 순서를 따름

            <특수 규칙>
            - 명확한 도구 명칭이 없지만 문맥상 유추 가능한 경우(예: "클램프" → Hemostat)는 리스트 내 항목으로 최대한 추론해 반환하세요.
            - 다수의 도구가 동시에 등장할 경우 각각에 대해 정확히 매칭하여 순서대로 출력하세요.
            - 도구 명칭이 한글로 등장할 경우 영어로 변환하여 출력하세요.(단어의 첫글자는 대문자로 변환, 나머지는 소문자로 변환, 예: "메스" → "Scalpel")
            - 도구와 무관한 단어(예: "주세요", "필요해", "건네줘" 등)는 무시하고 도구명만 추출하세요.
            - 동일한 도구가 여러 번 등장해도 한 번만 추출하세요.

            <예시>
            - 입력: "mess"
            출력: Scalpel
            
            - 입력: "mass"
            출력: Scalpel   

            - 입력: "math"
            출력: Scalpel

            - 입력: "매스"
            출력: Scalpel

            - 입력: "메스"
            출력: Scalpel 

            - 입력: "맥스"
            출력: Scalpel

            - 입력: "멕스"
            출력: Scalpel

            - 입력: "max"
            출력: Scalpel

            - 입력: "suction"
            출력: Yankauer 
                        
            - 입력: "sucsion"
            출력: Yankauer 
            
            - 입력: "석션"
            출력: Yankauer
            
            - 입력: "썩션"
            출력: Yankauer
            
            - 입력: "석쎤"
            출력: Yankauer

            - 입력: "썩쎤"
            출력: Yankauer
                                
            - 입력: "clamp"
            출력: Hemostat                   
             
            - 입력: "클램프"
            출력: Hemostat 
                               
            - 입력: "클렘프"
            출력: Hemostat      
                          
            - 입력: "Mayo"
            출력: Mayo_metz
                                
            - 입력: "메이요"
            출력: Mayo_metz
                                
            - 입력: "매이요"
            출력: Mayo_metz
                                
            - 입력: "마요"
            출력: Mayo_metz
                                
            - 입력: "마이요"
            출력: Mayo_metz
                                
            - 입력: "메이요"
            출력: Mayo_metz  
                              
            - 입력: "마이오"
            출력: Mayo_metz
                            
            - 입력: "마오"
            출력: Mayo_metz
                         
            - 입력: "forcep"
            출력: Forceps                    
            
            - 입력: "porcep"
            출력: Forceps

            - 입력: "forsep"
            출력: Forceps                    
            
            - 입력: "porsep"
            출력: Forceps

            - 입력: "poorsep"
            출력: Forceps

            - 입력: "poorcep"
            출력: Forceps

            - 입력: "poursep"
            출력: Forceps

            - 입력: "pourcep"
            출력: Forceps

            - 입력: "foorsep"
            출력: Forceps

            - 입력: "foorcep"
            출력: Forceps 

            - 입력: "foursep"
            출력: Forceps

            - 입력: "fourcep"
            출력: Forceps 

            - 입력: "포셉"
            출력: Forceps          
                      
            - 입력: "포샙"
            출력: Forceps

            - 입력: "뽀셉"
            출력: Forceps          
                      
            - 입력: "뽀샙"
            출력: Forceps

            - 입력: "폴셉"
            출력: Forceps          
                      
            - 입력: "폴샙"
            출력: Forceps

            - 입력: "뽈셉"
            출력: Forceps          
                      
            - 입력: "뽈샙"
            출력: Forceps

            - 입력: "포쎕"
            출력: Forceps          
                      
            - 입력: "포쌥"
            출력: Forceps

            - 입력: "뽀쎕"
            출력: Forceps          
                      
            - 입력: "뽀쌥"
            출력: Forceps

            - 입력: "폴쎕"
            출력: Forceps          
                      
            - 입력: "폴쌥"
            출력: Forceps

            - 입력: "뽈쎕"
            출력: Forceps          
                      
            - 입력: "뽈쌥"
            출력: Forceps

            - 입력: "Camera"
            출력: wound

            - 입력: "kamruh"
            출력: wound

            - 입력: "kamera"
            출력: wound
            
            - 입력: "카메라"
            출력: wound
            
            - 입력: "카맬아"
            출력: wound

            - 입력: "카멜아"
            출력: wound

            - 입력: "camara"
            출력: wound
            
            - 입력: "카매라"
            출력: wound
            
            - 입력: "camura"
            출력: wound
            
            - 입력: "카무라"
            출력: wound
            
            - 입력: "캐물아"
            출력: wound

            - 입력: "케물아"
            출력: wound
            
            - 입력: "캐무라"
            출력: wound

            - 입력: "케무라"
            출력: wound

            - 입력: "캐멀아"
            출력: wound

            - 입력: "케멀아"
            출력: wound

            - 입력: "캐머라"
            출력: wound

            - 입력: "케머라"
            출력: wound

            - 입력: "G-A"
            출력: Anesthesia

            - 입력: "지에이"
            출력: Anesthesia

            - 입력: "지애이"
            출력: Anesthesia

            - 입력: "쥐에이"
            출력: Anesthesia

            - 입력: "쥐애이"
            출력: Anesthesia

            - 입력: "즤에이"
            출력: Anesthesia

            - 입력: "즤애이"
            출력: Anesthesia

            - 입력: "지이에이"
            출력: Anesthesia

            - 입력: "지이애이"
            출력: Anesthesia

            - 입력: "쥐이에이"
            출력: Anesthesia

            - 입력: "쥐이애이"
            출력: Anesthesia

            - 입력: "즤이에이"
            출력: Anesthesia

            - 입력: "즤이애이"
            출력: Anesthesia

            - 입력: "메스와 클램프"
            출력: Scalpel Hemostat

            - 입력: " "
            출력:          

            - 입력: "종료"
            출력: 종료

            - 입력: "종뇨"
            출력: 종료

            - 입력: "종노"
            출력: 종료

            - 입력: "jongro"
            출력: 종료

            - 입력: "jongryo"
            출력: 종료

            - 입력: "완료"
            출력: 종료

            - 입력: "졸료"
            출력: 종료

            - 입력: "존료"
            출력: 종료

            - 입력: "졸료"
            출력: 종료    
             
            <사용자 입력>
            "{user_input}"
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = LLMChain(llm=self.llm, prompt=self.prompt_template)
        self.stt = STT(openai_api_key=openai_api_key)


        super().__init__("get_keyword_node")
        # 오디오 설정
        mic_config = MicConfig(
            chunk=12000,
            rate=48000,
            channels=1,
            record_seconds=5,
            fmt=pyaudio.paInt16,
            device_index=10,
            buffer_size=24000,
        )
        self.mic_controller = MicController(config=mic_config)
        # self.ai_processor = AIProcessor()

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request...")
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword
        )
        self.wakeup_word = WakeupWord(mic_config.buffer_size)

    def load_tool_list(self):
        json_path = os.path.join(package_path, "resource", "surgical_tools.json")
        with open(json_path, "r") as f:
            data = json.load(f)
        tool_names = list(data.values())
        return tool_names

    def extract_keyword(self, output_message):
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response["text"]

        # if "/" not in result:
        #     self.get_logger().warn(f"Unexpected format from LLM: {result}")
        #     return []

        # object_str, target_str = result.strip().split("/")
        
        # def normalize_object(word):
        #     word = word.strip().lower()
        #     for tool in self.tool_dict:
        #         if tool.lower() == word:
        #             return tool  # 원래 JSON에 저장된 도구 이름 포맷 그대로 반환
        #     return word.capitalize()  # 일치하는 게 없을 때만 fallback으로 capitalize

        # alias_dict 불러오기
        # alias_path = os.path.join(package_path, "resource", "alias_dict.json")
        # with open(alias_path, 'r', encoding='utf-8') as f:
        #     alias_dict = json.load(f)

        # 단어 정규화 함수
        # def normalize_word(word):
        #     word = word.strip().lower()
        #     print(f"word : {word}")
        #     return alias_dict.get(word, word.capitalize())
        
        object = [word for word in result.split()]
        # target = [normalize_word(word) for word in target_str.split()]


        print(f"llm's response: {object}")
        print(f"object: {object}")
        # print(f"target: {target}")
        return object
    
    def get_keyword(self, request, response):  # 요청과 응답 객체를 받아야 함
        try:
            print("open stream")
            self.mic_controller.open_stream()
            self.wakeup_word.set_stream(self.mic_controller.stream)
        except OSError:
            self.get_logger().error("Error: Failed to open audio stream")
            self.get_logger().error("please check your device index")
            return None

        while not self.wakeup_word.is_wakeup():
            pass

        # STT --> Keword Extract --> Embedding
        output_message = self.stt.speech2text()
        keyword = self.extract_keyword(output_message)

        self.get_logger().warn(f"{keyword}가 인식되었습니다. ")

        # 응답 객체 설정
        response.success = True
        response.message = " ".join(keyword)  # 감지된 키워드를 응답 메시지로 반환
        return response


def main():
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
