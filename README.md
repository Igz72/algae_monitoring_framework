# Algae Monitoring Framework

## Descrição do Projeto
Projeto desenvolvido para a disciplina de Robôs Móveis Autônomos, ofertada pelo Departamento de Computação da Universidade Federal de São Carlos (UFSCar). Este framework foi descrito no artigo `Unmanned Aerial Vehicle Framework for Algae Monitoring`, publicado na conferência Latin American Robotics Symposium/Brazilian Robotics Symposium (LARS/SBR), disponível em [IEEE Xplore](https://ieeexplore.ieee.org/document/9605379).

## Tópicos
- [Instalação](#instalação)
- [Desenvolvimento](#desenvolvimento)
	- [Organização dos pacotes](#organização-dos-pacotes)
	- [Criação dos pacotes](#criação-dos-pacotes)
	- [Criação de mensagens, serviços e ações](#criação-de-mensagens-serviços-e-ações)
	- [Criação de nós e módulos em Python](#criação-de-nós-e-módulos-em-python)
		- [Nós](#nós)
		- [Módulos](#módulos)

## Instalação
Para instalar o framework, esse repositório deve ser clonado na pasta `src` de um workspace catkin. Em seguida, a partir da pasta do workspace, deve-se executar os comandos:

    catkin_make
    source devel/setup.bash

- Sempre que houver alguma modificação nos pacotes, deve-se executar novamente os comandos indicados acima.
- Antes de iniciar o framework a partir de um novo terminal, ou após alguma modificação, deve-se executar o comando `source devel/setup.bash`.

## Desenvolvimento

### Organização dos pacotes
Os pacotes foram organizados seguindo as recomendações presentes em [Package Organization For a ROS Stack [Best Practices]](https://roboticsbackend.com/package-organization-for-a-ros-stack-best-practices/).
- `algae_monitoring_msgs`: Define as mensagens, os serviços e as ações utilizadas pelo framework;
- `algae_monitoring_bringup`: Contém as configurações e os arquivos launch que inicializam os nós;
- `algae_monitoring_control`: Responsável por controlar o UAV;
- `algae_monitoring_coverage`: Responsável por calcular a rota de coverage;
- `algae_monitoring_detector`: Responsável por detectar as algas;
- `algae_monitoring_master`: Responsável por gerenciar todas as operações.

### Criação dos pacotes
Os pacotes foram criados de acordo com o tutorial [Creating a ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage). Os seguintes comandos foram utilizandos:

	catkin_create_pkg algae_monitoring_msgs std_msgs sensor_msgs actionlib_msgs message_generation message_runtime
	
	catkin_create_pkg algae_monitoring_bringup
	
	catkin_create_pkg algae_monitoring_control algae_monitoring_msgs actionlib_msgs

	catkin_create_pkg algae_monitoring_coverage algae_monitoring_msgs

	catkin_create_pkg algae_monitoring_detector algae_monitoring_msgs
	
	catkin_create_pkg algae_monitoring_master algae_monitoring_msgs actionlib_msgs

### Criação de mensagens, serviços e ações
As mensagens, serviços e ações foram criados no pacote `algae_monitoring_msgs`, seguindo os tutoriais [Creating a ROS msg and srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv) e [ROS Create Custom Action](https://roboticsbackend.com/ros-create-custom-action/). As dependências necessárias foram incluídas nos arquivos `CMakeLists.txt` e `package.xml` durante a criação do pacote, sendo necessário apenas acrescentá-las na função `catkin_package(CATKIN_DEPENDS ...)` do `CMakeLists.txt`. Além disso, foram criadas as pastas `msg`, `srv` e `action`.

- Novas mensagens devem ser criadas em arquivos na pasta `msg`. O nome de cada arquivo deve ser adicionado na função `add_message_files()` do `CMakeLists.txt`.
- Novos serviços devem ser criados em arquivos na pasta `srv`. O nome de cada arquivo deve ser adicionado na função `add_service_files()` do `CMakeLists.txt`.
- Novas ações devem ser criadas em arquivos na pasta `action`. O nome de cada arquivo deve ser adicionado na função `add_action_files()` do `CMakeLists.txt`.

### Criação de nós e módulos em Python
A configuração dos pacotes para utilizar código Python foi feita com base no tutorial [Installing Python scripts and modules](http://docs.ros.org/en/kinetic/api/catkin/html/howto/format2/installing_python.html).

#### Nós
Para a criação de nós, a pasta `scripts` foi adicionada aos pacotes. A documentação sugere que os nós contenham pouco código, e que as implementações sejam importadas dos módulos. Além disso, para que sejam instalados, o nome dos arquivos devem ser adicionados na função `catkin_install_python()` no `CMakeList.txt`.

#### Módulos
Para a criação de módulos, a pasta `src/nome_do_pacote` foi adicionada aos pacotes. Nessa pasta foi adicionado o arquivo vazio `__init__.py`. Além disso, para que essa pasta seja reconhecida, o arquivo `setup.py` foi adicionado aos pacotes indicando esse endereço, e a função `catkin_python_setup()` foi acrescentada ao `CMakeList.txt`.
