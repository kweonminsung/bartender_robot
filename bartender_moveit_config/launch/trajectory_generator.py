import pandas as pd
import numpy as np

def generate_full_trajectory(start_disp_file, disp_user_file, output_file):
    # 1. 파일 읽기
    df_sd = pd.read_csv(start_disp_file)
    df_du = pd.read_csv(disp_user_file)
    
    # 컬럼 이름 가져오기 (time, joint1, joint2, ...)
    cols = df_sd.columns.tolist()
    joint_cols = cols[1:]
    
    # --- Step 1: 초기 10초 대기 (All Zeros) ---
    # 0.0초부터 10.0초까지 0.1초 간격
    t_init = np.arange(0.0, 10.05, 0.1)
    data_init = np.zeros((len(t_init), len(cols)))
    data_init[:, 0] = t_init
    df_init = pd.DataFrame(data_init, columns=cols)
    
    # --- Step 2: Start -> Dispenser 이동 ---
    # df_sd의 시간 축을 10.0초 뒤로 미룸
    # 첫 번째 점(0.0초)은 df_init의 마지막 점과 겹치므로 제외하거나 자연스럽게 연결
    df_sd_shifted = df_sd.iloc[1:].copy() # 0.0초 제외
    last_init_time = df_init.iloc[-1]['time']
    # df_sd의 시간 간격을 유지하며 shift
    df_sd_shifted['time'] = df_sd_shifted['time'] + last_init_time
    
    # --- Step 3: Dispenser에서 50초 대기 ---
    last_sd_row = df_sd_shifted.iloc[-1]
    last_sd_time = last_sd_row['time']
    last_sd_pose = last_sd_row[joint_cols].values
    
    t_wait = np.arange(last_sd_time + 0.1, last_sd_time + 50.05, 0.1)
    data_wait = np.zeros((len(t_wait), len(cols)))
    data_wait[:, 0] = t_wait
    data_wait[:, 1:] = last_sd_pose # 마지막 자세 유지
    df_wait = pd.DataFrame(data_wait, columns=cols)
    
    # --- Step 4: Dispenser -> User 이동 ---
    last_wait_time = df_wait.iloc[-1]['time']
    df_du_shifted = df_du.iloc[1:].copy() # 0.0초 제외
    df_du_shifted['time'] = df_du_shifted['time'] + last_wait_time
    
    # --- Step 5: User -> Start 복귀 (5초간 인터폴레이션) ---
    last_du_row = df_du_shifted.iloc[-1]
    last_du_time = last_du_row['time']
    last_du_pose = last_du_row[joint_cols].values
    
    # 복귀 시간 5초 설정
    t_return = np.arange(last_du_time + 0.1, last_du_time + 5.05, 0.1)
    steps = len(t_return)
    
    data_return = np.zeros((steps, len(cols)))
    data_return[:, 0] = t_return
    
    # 각 관절별로 현재 위치에서 0까지 선형 보간 (Linear Interpolation)
    for i, col in enumerate(joint_cols):
        start_val = last_du_pose[i]
        end_val = 0.0
        # linspace로 부드럽게 연결
        interp_vals = np.linspace(start_val, end_val, steps + 1)[1:]
        data_return[:, i+1] = interp_vals
        
    df_return = pd.DataFrame(data_return, columns=cols)
    
    # --- 전체 병합 ---
    df_final = pd.concat([df_init, df_sd_shifted, df_wait, df_du_shifted, df_return], ignore_index=True)
    
    # CSV 저장
    df_final.to_csv(output_file, index=False)
    print(f"Generated {output_file} with {len(df_final)} points.")

# 3가지 케이스 생성 실행
generate_full_trajectory('./start_dispenser1.csv', './dispenser1_user.csv', './case1_final.csv')
generate_full_trajectory('./start_dispenser2.csv', './dispenser2_user.csv', './case2_final.csv')
generate_full_trajectory('./start_dispenser3.csv', './dispenser3_user.csv', './case3_final.csv')