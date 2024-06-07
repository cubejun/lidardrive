# lidardrive
라이다 스캔 데이터를 publish하는 sllidar_node 실행파일과

라이다 스캔 데이터를 subscribe해 평면도를 그리고 평면도에 로봇의 정면 부분을 왼쪽과 오른쪽으로 관심영역을 나누어 레이블링을 해 로봇 중심과 최단거리인 장애물을 구하고 정면방향과 각도 차이를 에러값으로 계산해 주행로봇으로 에러값 publish 하는 vm 실행파일과

에러값을 subscribe 해 다이나믹셀을 제어하는 jetson 실행파일을 
하나의 lidardrive패키지로 만들었다.
# 동작영상
https://youtu.be/PftGyO2YzhE


https://youtube.com/shorts/ICAtPBVJINk


https://youtu.be/zSRObVfqriM?si=yEafjMEmHOdBgQ99


https://youtu.be/ow-M28vpyZQ
