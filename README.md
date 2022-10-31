# Capstone
2022 창의 설계 경진대회

## 영상 처리를 이용한 건물 내 스마트 배달 시스템
학교, 사무실 등 건물 내에서 택배를 전달해주는 로봇이다. 영상처리를 이용하여 운송장의 주소를 인식하고, 해당 주소를 목적지로 설정하여 주행을 시작한다. 천장에 그려져 있는 차선을 인식하여 모터를 제어하고, 자체적인 주소 체계를 구축하여 정확한 위치까지 택배를 배달해준다.  


## 구현 원리
1. 배달로봇 주행 시스템
    - 영상처리를 이용한 차선 인식, RGB scale에서의 한계를 극복하기 위해 HLS color scale로 변환
    - Hough Transform 기반의 소실점 추출을 활용한 주행 시스템 개발
    - RaspberryPi로 읽은 정보를 Socket 통신으로 받아와 배달 로봇의 목적지 설정  
2. Image Processing과 OCR-terseract를 이용한 텍스트 인식 및 Socket 통신
    - 서버(RaspberryPi)에서 클라이언트(PC)로 이미지를 전달하기 위해 Socket 통신 사용  
    - OCR-tesseract을 이용하여 주소값을 출력하고, 모터 제어를 위한 입력값으로 할당
    - 배달지에 도착한 뒤, 카메라로 촬영한 문의 이미지를 Socket 통신으로 다시 클라이언트에게 전달  
    - 상자의 contour를 추출하여 상자가 관심영역에 들어왔을 때에만 주소를 읽도록 하여 정확성을 높임


## 시연 영상
...

## Version 관리
Rasbian OS x.x.x  
Python x.x.x

## Hardware
Raspberry Pi 4 Computer Model B 8GB RAM  
dc 모터 드라이버 : L298n

## 상세 내용
<프로젝트 종료 후 Notion 링크(외부인 편집 불가능 모드) 업로드 예정>
