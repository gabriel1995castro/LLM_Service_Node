#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

class LLMServiceClient : public rclcpp::Node
{
public:
    LLMServiceClient() : Node("llm_service_client")
    {
        // Criando o cliente para o serviço
        client_ = this->create_client<std_srvs::srv::SetBool>("/admittance_modulator/set_validation");

        // Aguarda o serviço estar disponível
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Aguardando o serviço...");
        }

        // Criar e enviar a solicitação
        send_request(true);  // Envia um comando para ativar (true)
    }

private:
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;

    void send_request(bool activation_flag)
    {
        // Criando uma solicitação
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = activation_flag;

        // Enviando a solicitação e configurando o callback de resposta
        auto future = client_->async_send_request(request, 
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response) {
                if (response.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "Serviço executado com sucesso: %s", response.get()->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Falha ao executar serviço.");
                }
            });
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LLMServiceClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
