#!/usr/bin/env python3
from langchain_ollama import ChatOllama
from langchain.callbacks.streaming_stdout import StreamingStdOutCallbackHandler
from langchain.callbacks.manager import CallbackManager
from langchain_core.prompts import ChatPromptTemplate
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


callback_manager = CallbackManager([StreamingStdOutCallbackHandler()])


LLM_model = ChatOllama(
    model="qwen2.5:32b",  
    temperature=0.4,      
    num_predict=1200,     
    callback_manager=callback_manager,  
    seed=None,            
)


prompt = ChatPromptTemplate.from_messages(
    [
        ("system", "You are a helpful assistant."),  
        ("human", "{input}"),                      
    ]
)


chain = prompt | LLM_model

class OllamaLLM(Node):
    def __init__(self) -> None:
        super().__init__("LLM_Control_Node")
        self.publisher_ = self.create_publisher(String, 'ollama_node', 10)

        self.client_ = self.create_client(SetBool, "/admittance_modulator/set_validation")

        while not self.client_.wait_for_service(timeout_sec=1.0):
           
            self.get_logger().warn("Aguardando o serviço /admittance_modulator/set_validation...")

    def send_service_request(self, activation_flag: bool):
        """ Envia um comando para ativar/desativar o `validation_flag_` """
        request = SetBool.Request()
        request.data = activation_flag

        future = self.client_.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        """ Callback para processar a resposta do serviço """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Serviço executado: {response.message}")
            else:
                self.get_logger().warn("Falha ao executar serviço.")
        except Exception as e:
            self.get_logger().error(f"Erro na chamada do serviço: {str(e)}")
    
    def conversation(self):
        context = ""  

        while True:
            user_input = input("\nYou: ")
            if user_input.lower() == 'exit':
                print("Goodbye!")
                break

            print("Bot: ", end="")
            result = chain.invoke({"input": user_input})
            bot_response = result.text if hasattr(result, 'text') else str(result)  
            context += f"\nUser: {user_input}\nBot: {bot_response}" 
            
            self.publisher_.publish(String(data=bot_response))
           
            if "ativar modo seguidor" in user_input.lower():
                self.send_service_request(True)
           
            elif "desativar modo seguidor" in user_input.lower():
                self.send_service_request(False)

def main(args=None):
    rclpy.init(args=args)
    node = OllamaLLM()
    node.conversation()  
    rclpy.spin(node)  
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
