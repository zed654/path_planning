
	코드 구성 설명 (2020.07.08)

	< Hybrid A* + Artificial Potential Field 코드. >

		- Hybrid A*의 차량 Kinematic Model을 고려했다는 장점과 Mutli-Goal을 추종하기 위한 방안으로 Potential Field를 엮은 알고리즘.
		- 차량이 Waypoints들을 추종하며 Obstacle을 회피하는 Local Path를 생성하기 위한 목적으로 설계
		- 코드 구성 요약으로, Hybrid A*의 Cost부분을 Artificial Potential Field로 대체함.


			알고리즘 장점 : 차량의 Kinematic Model이 고려됨 (주행 가능 영역 위주로 주행)	
			알고리즘 단점 : 속도 (A* 보다 느림), 파라미터의 복잡성(length_arc, radius, obstacles, Waypoints, ...)

-------------------------------------------------------------------------------------------------------------------------------------
		- Hybrid A* 알고리즘 구성 설명

			- Hybrid A*의 구성은 현재위치(xn, yn)와 현재 위치에서 다음으로 향하기 위한 회전중심위치(xc, yc)임.
				-> 회전중심위치(xc, yc)는 현재위치(xn, yc)에서의 차량의 헤딩 정보를 내포한다고 보면 됨

			- New Node를 증식시킬 때, 생성할 Node의 개수를 맞춰줘야함. Node의 개수는 현재 위치(xn, yn)과 회전중심위치(xc, yc)의 길이(반지름, radius)에 의존함.

			- 증식된 Node는 현재위치(xn, yn)를 기준으로 회전중심위치(xc, yc)로부터 회전시킨 호의 길이(length_arc)와 반지름(radius)에 의존함. 
				만일 호의 길이(length_arc)가 너무 크면, 다음 증식된 노드가 한 바퀴 돌아서 원하는 방향의 뒤편에 위치할 수 있음.
				-> 전진해야하는 차량을 후진하게 하는 효과.

			- 반면 length_arc가 너무 작으면, 증식된 노드의 Open/Closed 상태가 부모 노드에 의존하게 됨. 
				즉, 노드 증식 후 부모 노드는 Closed가 true가 될 것인데, 증식된 노드의 Closed값이 false가 아닌 true가 되는 문제.
				

				-> 따라서, 적절한 length_arc 값이 필요함. (약 1.6 정도 추천)

 -------------------------------------------------------------------------------------------------------------------------------------
		- Artificial Potential Field 알고리즘 구성 설명

			- Hybrid A*의 Cost 계산 부분을 대체함.

			- Waypoints 들을 Attractive Potential로 정의
			- Obstacles 들을 Repulsive Potential로 정의
			- Potential은 EuclideanDistance로 구성 (exp로 구성 시, 적용되는 범위가 너무 협소하기 때문)
	

