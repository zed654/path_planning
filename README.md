
- 구동 환경 
	
	1. Ubuntu 16.04 or 18.04

	2. ROS

	3. OpenCV (ROS 설치 시 자동 설치)

- 실행 방법


	1. A*

	optimization_class_ws 폴더의 경로에서 터미널 창에 다음 명령어 입력

		1. catkin_make
	
		2. source devel/setup.bash

		3. rosrun a_star_algorithm_pkg a_star_algorithm_pkg [img_num]

		   ex. rosrun a_star_algorithm_pkg a_star_algorithm_pkg 5





	2. MGPF-Hybrid A*

	optimization_class_ws 폴더의 경로에서 터미널 창에 다음 명령어 입력

		1. catkin_make

		2. source devel/setup.bash

		3. rosrun hybrid_a_star_algorithm_pkg hybrid_a_star_algorithm_pkg_exe [img_num]

		   ex. rosrun hybrid_a_star_algorithm_pkg hybrid_a_star_algorithm_pkg_exe 5


		- 명령어 변수 설명

			1. img_num은 실행할 Obstacle Images로 1~7의 숫자 입력 가능




	3. Genetic Algorithm

		아직 Path Planning으로 구현하지 않음.

		테스트 코드 rosrun genetic_algorithm_pkg genetic_algorithm_pkg_exe


	4. PSO Algorithm

		Not working well ,,

 -----------------------------------------------------

 - Version

	1. 2020.03.16 A* 알고리즘 추가

	2. 2020.06.05 Hybrid A* 알고리즘 추가

	3. 2020.08.26 Hybrid A*를 MGPF-Hybrid A* (Multi-Goals Potential Field Hybrid A*)로 교체

	4. 2021.12.08 PSO Path Planning Algorithm is added. But, not working well. It has various BUG :)

