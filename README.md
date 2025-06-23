# MuseuRover: Controle Remoto e Live Feed para Leo Rover

Esta aplicação oferece uma interface completa para controlar um **Leo Rover** remotamente, combinando o uso de um **controle joystick** para movimentação intuitiva e um **live feed da câmera** para navegação visual. Desenvolvido para facilitar visitas remotas a museus ou qualquer ambiente, ele proporciona uma experiência imersiva ao usuário.

---

## Funcionalidades Principais

* **Controle do Leo Rover via Joystick:**
    * Utiliza a biblioteca `Pygame` para detectar e interpretar comandos de um joystick conectado.
    * Traduz os inputs do joystick em comandos de movimentação para o Leo Rover, permitindo direção, aceleração e frenagem precisas.
* **Visualização da Câmera em Tempo Real (Live Feed):**
    * Exibe o stream de vídeo da câmera do Leo Rover em uma interface PyQt5.
    * Conecta-se ao **tópico ROS** da câmera do rover (`/leo_main_duque/camera/image_raw/compressed`) para receber as imagens comprimidas.
    * **Monitoramento de Conexão:** Um indicador de status em tempo real informa se a câmera está transmitindo ou se há perda de conexão, garantindo que o usuário esteja sempre ciente do estado do sistema.
* **Interface Gráfica Intuitiva (PyQt5):**
    * Janela dedicada para a exibição do live feed, com um título claro ("Museum Remote Visit - Live Feed") e um status de conexão visível.
    * Design otimizado para uma visualização clara e focada na transmissão de vídeo.

---

## Estrutura do Código

O código é dividido em duas classes principais para modularidade e gerenciamento de threads:

* **`ImageSubscriber(QThread)`:**
    * Responsável por se inscrever no tópico ROS da câmera (`CompressedImage`).
    * Converte as mensagens `CompressedImage` recebidas em objetos `QImage` compatíveis com PyQt5.
    * Emite um sinal (`image_signal`) com a `QImage` convertida para atualização na interface gráfica.
    * Possui um mecanismo interno para verificar a atividade do stream e emitir um sinal de status (`connection_status_signal`), informando se o rover está "Conectado / Transmitindo" ou "Desconectado / Sem Stream".
    * Executa em uma **thread separada** para não bloquear a interface principal do PyQt5.
* **`ImageViewer(QWidget)`:**
    * A classe principal da interface gráfica (GUI).
    * Cria e gerencia a janela principal, o `QLabel` para exibir as imagens da câmera e o `QLabel` para o status de conexão.
    * Conecta os sinais da `ImageSubscriber` aos seus próprios slots (`update_image` e `update_status`) para exibir o vídeo e o status.
    * Escala automaticamente as imagens recebidas para se ajustarem ao tamanho do `QLabel` de vídeo, mantendo a proporção.
    * Gerencia o ciclo de vida do `ImageSubscriber`, garantindo que a thread seja encerrada corretamente ao fechar a aplicação.

---

## Requisitos e Compatibilidade

* **Sistema Operacional:** Linux (com ROS instalado e configurado).
* **Python:** Versão **3.8.10**. Esta versão foi escolhida devido à compatibilidade com as bibliotecas `PyQt5` e `Pygame`, garantindo um ambiente de execução estável para ambas as funcionalidades (visualização da câmera e controle por joystick).
* **ROS (Robot Operating System):** Essencial para a comunicação com o Leo Rover.
* **Bibliotecas Python:**
    * `rospy` (para ROS em Python)
    * `PyQt5` (para a interface gráfica)
    * `sensor_msgs` (para tipos de mensagem ROS)
    * `cv_bridge` (para conversão de imagem entre ROS e OpenCV)
    * `opencv-python` (cv2, para manipulação de imagens)
    * `pygame` (para o controle do joystick – *observe que a parte do código `pygame` não está incluída neste arquivo, mas é essencial para a funcionalidade de controle*).

---

## Configuração e Execução

### Pré-requisitos

1.  **Instale ROS:** Certifique-se de ter uma instalação funcional do ROS (preferencialmente Noetic, compatível com Python 3.8) em sua máquina.
2.  **Crie um Workspace ROS:** Se ainda não tiver um, crie um workspace Catkin:
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    ```
3.  **Clone o Repositório:** Clone este repositório para o diretório `src` do seu workspace ROS:
    ```bash
    cd ~/catkin_ws/src
    git clone [https://github.com/Laboratorio-de-Informatica-Industrial/MuseuRover.git](https://github.com/Laboratorio-de-Informatica-Industrial/MuseuRover.git)
    ```
4.  **Instale as Dependências Python:**
    ```bash
    pip install PyQt5 opencv-python pygame
    pip install ros-noetic-cv-bridge # Ou a versão correspondente ao seu ROS
    ```
    *Certifique-se de que `pip` esteja configurado para a versão 3.8.10 do Python.*

5.  **Compile o Workspace (se houver outros pacotes ROS):**
    ```bash
    cd ~/catkin_ws/
    catkin_make
    source devel/setup.bash
    ```

### Execução da Aplicação

1.  **Inicie o ROS Master:**
    ```bash
    roscore
    ```
2.  **Certifique-se de que o Leo Rover está Online e Transmitindo:**
    O Leo Rover deve estar ligado e publicando o stream de câmera no tópico `/leo_main_duque/camera/image_raw/compressed`. Você pode verificar isso com:
    ```bash
    rostopic list
    rostopic hz /leo_main_duque/camera/image_raw/compressed
    ```
3.  **Execute a Aplicação:**
    Navegue até o diretório do script e execute-o:
    ```bash
    cd ~/catkin_ws/src/MuseuRover/pygame_joycam/RoverMuseu # Ou o caminho correto para onde seu script está
    python3 main_app.py # Substitua main_app.py pelo nome do seu script principal
    ```
    Ou, se você o configurou como um nó ROS:
    ```bash
    rosrun seu_pacote_ros_aqui seu_script_aqui.py
    ```

---

## Solução de Problemas Comuns

* **Tela preta na câmera / Pixels pretos nas bordas:**
    * Verifique se o tópico da câmera (`/leo_main_duque/camera/image_raw/compressed`) está realmente ativo e publicando mensagens.
    * Confirme se a conversão `CvBridge` está funcionando corretamente (verifique os logs do ROS).
    * **Sugestão de Correção:** Na classe `ImageSubscriber`, altere a linha que define `bytes_per_line` para:
        ```python
        bytes_per_line = rgb_image.strides[0]
        ```
        Isso garante a correta interpretação dos dados da imagem ao criar a `QImage`, corrigindo problemas de alinhamento que podem causar bordas pretas.
* **Erros de importação:**
    * Certifique-se de que todas as bibliotecas Python necessárias (`PyQt5`, `opencv-python`, `pygame`, `rospy`, `cv_bridge`) estão instaladas para a **versão específica do Python 3.8.10**.
* **"ROS Initialization Error":**
    * Confirme que o `roscore` está em execução.
    * Verifique suas variáveis de ambiente ROS (`ROS_MASTER_URI`, `ROS_IP`).
