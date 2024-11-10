# DooSan-Robotics-TTL

* projectsrv.zip을 다운 받을 것
* 소스 파일의 py를 보면 이미지나 오디오 파일들의 경로 이름이 설정된 것들이 있는데 이것들은 자기 환경에 맞게 바꿔줘야함

### projectsrv
---

키오스크와 키친을 띄우고 서로 서비스 방식으로 통신함
이후 로봇이 받아야할 정보를 토픽으로 전송함

```bash
ros2 run projectsrv kitchen 
```

```bash
ros2 run projectsrv kiosk 
```

### ttb_gui
---

터틀봇 인터페이스

주문 정보를 받아서 네비게이션에게 전달하는 패키지 입니다. 

```bash
ros2 run ttb_gui delivery_order.py 
```

토픽 pub 

```bash
 ros2 run  ttb_gui delivery_topic.py 
```

로봇 액션 서버
```bash
 ros2 run ttb_gui delivery_server.py 
```


![image](https://github.com/user-attachments/assets/71415ad5-3b5a-4440-9a47-3e7502879c76)
