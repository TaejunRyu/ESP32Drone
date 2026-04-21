import serial
import time

# --- 설정 ---
port = '/dev/ttyUSB2'  # 윈도우는 'COMx', 맥/리눅스는 '/dev/ttyUSB0' 등 확인 필요
baudrate = 115200
timeout = 1

def process_data(raw_line):
    """데이터 가공 함수 (프로젝트에 맞게 수정)"""
    try:
        # 1. 바이트를 문자열로 변환 및 공백 제거
        data_str = raw_line.decode('utf-8').strip()
        
        if not data_str:
            return None

        # 2. 예: "12.5, 23.1, 45.0" 처럼 콤마로 구분된 데이터 분리
        values = data_str.split(',')
        
        # 3. 숫자형으로 변환 (예: float 리스트)
        numeric_data = [float(v) for v in values]
        
        return numeric_data
    except Exception as e:
        print(f"가공 에러: {e}")
        return None


# --- 실행 ---
try:
    ser = serial.Serial(port, baudrate, timeout=timeout)
    print(f"연결 성공: {port}")
    time.sleep(2)  # 포트 열린 후 안정화 대기

    while True:
        if ser.in_waiting > 0:
            # 1. 시리얼 한 줄 읽기
            line = ser.readline()
            
            # 2. 데이터 가공
            processed = process_data(line)
            
            if processed:
                # 3. 활용 (출력, DB 저장, 그래프 그리기 등)
                print(f"수신 데이터: {processed}")
                # 예: 롤/피치 값 추출
                # roll = processed[0]
                # pitch = processed[1]

except serial.SerialException as e:
    print(f"포트 연결 실패: {e}")
except KeyboardInterrupt:
    print("\n프로그램을 종료합니다.")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("포트가 닫혔습니다.")
