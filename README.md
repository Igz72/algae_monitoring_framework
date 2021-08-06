# Utilização

Vídeo ilustrando o funcionamento do serviço de path planning: https://www.youtube.com/watch?v=Q31L-s6_7aA

Vídeo ilustrando o funcionamento do nó master em uma área reduzida de coverage: https://www.youtube.com/watch?v=ipliQb9jSWc

# Desenvolvimento
## Criação do pacote ([Creating a ROS Package][criar_pacote])
O pacote [artigo_framework][artigo_framework] foi criado utilizando o comando:

	catkin_create_pkg artigo_framework std_msgs rospy

## Criação de serviços ([Creating a ROS msg and srv][criar_servico])
O arquivo [srv/PathPlanningCoverage.srv][path_planning_coverage] foi criado como base para a criação do serviço que calcula o _path planning_ para o _coverage_. As variáveis utilizadas pelo _request_ e _response_ são separadas pelo delimitador `---`.

	float64 mapa_inicio_x
	float64 mapa_inicio_y
	float64 mapa_largura
	float64 mapa_altura
	float64 camera_largura
	float64 camera_altura
	---
	float64[] caminho_x
	float64[] caminho_y

O nome do arquivo criado deve ser adicionado em [CMakeList.txt][CMakeList_pacote]:

	add_service_files(
	  FILES
	  PathPlanning.srv
	)

## Criação de módulos e nós em Python ([Installing Python scripts and modules][modulos_python])
### Módulos
Para a utilização de módulos criados em Python, a pasta [src/artigo_framework][src] foi criada contendo o arquivo vazio `__init__.py`. Além disso, adicionou-se o arquivo [setup.py][setup] na raiz do pacote:

	## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

	from setuptools import setup
	from catkin_pkg.python_setup import generate_distutils_setup

	# fetch values from package.xml
	setup_args = generate_distutils_setup(
	    packages=['artigo_framework'],
	    package_dir={'': 'src'})

	setup(**setup_args)

Para terminar a configuração, a seguinte linha foi adionada ao [CMakeList.txt][CMakeList_pacote]:

	catkin_python_setup()

O novos módulos escritos em Python devem ser criados em [src/artigo_framework][src], e então o pacote deve ser recompilado.

### Nós
Os nós escritos em Python devem ser colocados na pasta [scripts][scripts]. A documentação sugere que os nós contenham pouco código, e que as implementações sejam importadas dos módulos. Além disso, cada novo nó deve ser instalado. Para isso, deve-se adicionar seu caminho relativo em [CMakeList.txt][CMakeList_pacote]:

	catkin_install_python(PROGRAMS
	  scripts/path_planning_server.py
	  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

[artigo_framework]: <https://github.com/Igz72/artigo_framework>
[CMakeList_pacote]: <https://github.com/Igz72/artigo_framework/blob/main/CMakeLists.txt>
[path_planning_coverage]: <https://github.com/Igz72/artigo_framework/blob/main/srv/PathPlanningCoverage.srv>
[src]: <https://github.com/Igz72/artigo_framework/tree/main/src/artigo_framework>
[setup]: <https://github.com/Igz72/artigo_framework/blob/main/setup.py>
[scripts]: <https://github.com/Igz72/artigo_framework/tree/main/scripts>

[criar_pacote]: <http://wiki.ros.org/ROS/Tutorials/CreatingPackage>
[criar_servico]: <http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv>
[modulos_python]: <http://docs.ros.org/en/api/catkin/html/howto/format2/installing_python.html#installing-python-scripts-and-modules>
