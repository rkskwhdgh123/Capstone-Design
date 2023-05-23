# Capstone-Design
---
2023/03/08
---

아이디어 초안 결정

자율주행 서빙로봇, 여행 가이드 로봇, 재난 구조 로봇


---
2023/03/08 ~2023/03/23
---

아이디어 확정 : 스마트 물류로봇

회의 진행 , 개발환경 구축 시작 

발표용 구글 슬라이드 작성 
:https://docs.google.com/presentation/d/1B9U-DVyYhVAE5_ZT1NVz0CdpqMdhDtfcH2bLjb1DV3U/edit?hl=ko#slide=id.p


---
2023/03/24 ~2023/04/16
---

로봇 조립 완료,

카메라 동작 확인 완료,

액추에이터 제어 동작 확인 완료

---
2023/04/17 ~2023/04/27
---

라인 트래킹 제작중

---
블럭도
---

![image](https://github.com/rkskwhdgh123/Capstone-Design/assets/103232926/43de2a18-4244-4d7e-ae5e-e95b09216865)


---
PART1 leader following
---
2023.5.10
라인 검출후 라인을 추적하는 기능 구현

https://github.com/rkskwhdgh123/Capstone-Design/assets/103232926/8fff94f6-409b-4618-b475-776d9b8e1c47


현재 라인을 추적하는 로봇의 뒷 모습을 darknet yolo로 학습시키는 중



2023.5.23


라인을 추적하는 leader로봇을 추적하는 follower 로봇 제작 완료

https://github.com/rkskwhdgh123/Capstone-Design/assets/103232926/e6d4a6cc-97ca-4bc0-bf63-87d11bf72cc6



---
PART2 Lidar Mapping
---
2023.5.10
라이다센서에서 거리와 각도값 측정 후 장애물 그리기 기능 구현 중

https://github.com/rkskwhdgh123/Capstone-Design/assets/103232864/91e250ff-9a13-4015-adc2-b69d84470678

2023.5.22
라이다센서를 이용하여 장애물을 검출하고 회피하여 주행하는 기능 구현

https://github.com/rkskwhdgh123/Capstone-Design/assets/103232864/f647139a-15bb-44f7-9e9a-8c2ab82f7b22

라이다센서를 이용하여 가장 가까운 물체를 추적하는 기능 구현

https://github.com/rkskwhdgh123/Capstone-Design/assets/103232864/99cafe2c-4a62-4239-9608-56aa7764173e

---
PART3 Parking
---

2023.5.10

주차선 내부 주차 공간 인식중

https://github.com/rkskwhdgh123/Capstone-Design/assets/103232943/dad774a4-3685-47aa-879a-fbb1c3b7ed94

모터 제어전에 중심점을 정확하게 인식하기 위해 코드 수정중
