# Auto-parking-yolo-object-detection

ROS2 turtlebot3_navigation2 패키지를 사용하여 장애물을 회피하면서 목표지점까지 주행한후에 실행시키는 코드

기능 1 표지판 인식 및 보정  
: 표지판과 근접시 모터를 미세조종하여 표지판을 정확히 인식할수있도록 보정하는 기능  

![보정](https://github.com/rkskwhdgh123/Capstone-Design/assets/103232943/f3a65836-ed71-4219-bb6a-778a63824bfc)




기능 2 주차 가능 여부 판단  
yolo3_tiny와 yolo4_tiny를 비교하여
정확성이 높은 yolo4_tiny 가중치 파일을 사용했습니다.  

오른쪽 화살표 => 오른쪽 주차 가능 판단  
왼쪽 화살표 => 왼쪽 주차 가능 판단  
정지 => 주차 불가   

![주차방향인식](https://github.com/rkskwhdgh123/Capstone-Design/assets/103232943/bdda9841-416d-4f15-9b2b-54b7e179c2cd)



기능 3 주차   

정면카메라가 주차 공간을 발견하게되면  
주차 공간에 진입하고 정지합니다.  

![주차공간 진입 각도](https://github.com/rkskwhdgh123/Capstone-Design/assets/103232943/932f4506-4fdc-4713-859f-febbf531f770)    

</br>

![방향 지정](https://github.com/rkskwhdgh123/Capstone-Design/assets/103232943/f3f15da8-3e1c-425d-b305-1ced1efb3be8)




**정면 카메라 주차 공간 발견시 진입후 정지**


로봇이 회전중일때 이때는 정면 카메라가 주차 공간을 찾고있습니다.  
정면카메라가 주차 공간을 발견하게되면
주차 공간에 진입하고 정지합니다.


![주차공간 진입 ㅁ찐](https://github.com/rkskwhdgh123/Capstone-Design/assets/103232943/99ef8145-a8c3-4776-b7e7-4872bddabbb4)    


  
</br>
</br>

![진입 전방카메라](https://github.com/rkskwhdgh123/Capstone-Design/assets/103232943/5fa8ef74-9540-44f2-a7b0-fcb7bfcebd00)


</br>
</br>

**정확한 위치로 주차가 될수 있도록 보정**



다음은정확한 위치에 주차가 완료되도록 넣은 기능입니다.
코너 검출 알고리즘을 이용해 두개의 빨간색 원이 그려지면
중심좌표에 초록색 원을 그리게 됩니다.
그리고 잠시 뒤에 이 좌표를 고정한뒤 로봇을 후진하고
이 좌표가 중심점에 오도록 모터를 조정하고 다시 주차 공간에 진입해
정확한 위치에 들어갈수있도록 만들었습니다.


![ezgif com-resize (2)](https://github.com/rkskwhdgh123/Capstone-Design/assets/103232943/79abd75a-70b2-404e-92d7-9efe1902350e)

</br>
</br>

![주차보정](https://github.com/rkskwhdgh123/Capstone-Design/assets/103232943/46852f51-8705-4441-a3f5-70a425d7295c)


</br>
</br>




YouTube 영상 첨부
https://www.youtube.com/watch?v=tupCxr6IsIs
