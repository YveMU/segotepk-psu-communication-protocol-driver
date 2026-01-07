#!/usr/bin/env python3
"""
Segotep PSU Data Logger - 被动监听版
被动读取COM口传出的信息，不主动发送命令
"""

import serial
import time
import struct
import sys
from datetime import datetime
import os

# === 配置 ===
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
LOG_FILE = 'psu_data_passive.tsv'
LOG_INTERVAL = 1.0  # 记录到文件的最小间隔

# === 协议常量 ===
HEADER = b'\x55\x7E'
FOOTER = b'\xAE'

# === 数据存储 ===
psu_data = {
    'model': '',
    'serial': '',
    'pkt02': {},
    'pkt04': {},
}

def parse_packet_02(payload):
    """
    解析 0x02 电气参数包 (小端序 Little-Endian)
    
    最终确认的结构:
    Offset 01-02: +3.3V 电压 (mV)
    Offset 03-04: +5V 电压 (mV)
    Offset 05-06: +12V 电压 (mV)
    Offset 07-08: +5VSB 电压 (mV)
    Offset 09-10: +3.3V 电流 (mA)
    Offset 11-12: +5V 电流 (mA)
    Offset 13-14: +12V 电流 (mA)
    Offset 15-16: AC 频率 (x10 Hz)
    Offset 17-18: AC 电压 ADC原始值
    Offset 19-20: AC 电流 ADC原始值
    Offset 21-22: 功率因数相关值 (约7500左右)
    Offset 23-24: AC 输入电压 (x10 V)
    Offset 25-26: 风扇转速计数 (实际RPM = value * 30)
    """
    try:
        psu_data['pkt02']['raw_hex'] = payload.hex()
        psu_data['pkt02']['last_update'] = time.time()
        
        if len(payload) < 27:
            return False
        
        # 小端序解包 - 13个 uint16
        v = struct.unpack('<HHHHHHHHHHHHH', payload[1:27])
        
        # 存储原始值
        psu_data['pkt02']['raw_values'] = list(v)
        
        # 解析物理量
        psu_data['pkt02']['volt_3v3'] = v[0] / 1000.0
        psu_data['pkt02']['volt_5v'] = v[1] / 1000.0
        psu_data['pkt02']['volt_12v'] = v[2] / 1000.0
        psu_data['pkt02']['volt_5vsb'] = v[3] / 1000.0
        
        psu_data['pkt02']['curr_3v3'] = v[4] / 4096.0
        psu_data['pkt02']['curr_5v'] = v[5] / 4096.0
        psu_data['pkt02']['curr_12v'] = v[6] / 256.0
        
        psu_data['pkt02']['ac_freq'] = v[7] / 10.0
        psu_data['pkt02']['ac_volt_adc'] = v[8]  # ADC原始值
        psu_data['pkt02']['ac_curr_adc'] = v[9]  # ADC原始值
        psu_data['pkt02']['pf_factor'] = v[10]    # 功率因数相关
        psu_data['pkt02']['ac_voltage'] = v[11] / 10.0
        
        # 修正：风扇转速 = 寄存器值 * 30
        psu_data['pkt02']['fan_rpm'] = v[12] * 30
        
        # 计算DC功率
        p_3v3 = psu_data['pkt02']['volt_3v3'] * psu_data['pkt02']['curr_3v3']
        p_5v = psu_data['pkt02']['volt_5v'] * psu_data['pkt02']['curr_5v']
        p_12v = psu_data['pkt02']['volt_12v'] * psu_data['pkt02']['curr_12v']
        
        psu_data['pkt02']['power_3v3'] = p_3v3
        psu_data['pkt02']['power_5v'] = p_5v
        psu_data['pkt02']['power_12v'] = p_12v
        psu_data['pkt02']['power_dc_total'] = p_3v3 + p_5v + p_12v
        
        return True
        
    except Exception as e:
        print(f"[ERROR] 解析 0x02: {e}", file=sys.stderr)
        return False

def parse_packet_04(payload):
    """
    解析 0x04 扩展状态包 (大端序！经测试确认为大端)
    
    最终确认的结构:
    Offset 00: 包类型 (0x04)
    Offset 01: 风扇模式 (3=High/Custom, 4=Auto/Ramp, 5=Mute, 6=OC, 8=Clean)
    
    以下为大端序 uint16:
    Offset 02-03: 主温度 High (x10 °C) [CSV v00]
    Offset 04-05: 风扇目标PWM/周期 [CSV v01]
    Offset 06-07: 状态标志 (356=低负载, 612=高负载) [CSV v02]
    Offset 08-09: 固定值256 [CSV v03]
    Offset 10-11: 固定值8132 [CSV v04]
    Offset 12-13: 辅助温度 Low/MOS (x10 °C) [CSV v05]
    Offset 14-15: 内部温度或计数 [CSV v06]
    Offset 16-17: AC输入功率 (x10 W) *** [CSV v07]
    Offset 18-19: 固定值565 [CSV v08]
    Offset 20-21: 内部计数器 [CSV v09]
    Offset 22-23: 未知参数 [CSV v10]
    Offset 24-25: 气流温度 Air (x100 °C) [CSV v11]
    Offset 26-27: 未知参数 [CSV v12]
    """
    try:
        psu_data['pkt04']['raw_hex'] = payload.hex()
        psu_data['pkt04']['last_update'] = time.time()
        
        if len(payload) < 28:
            return False
        
        # 存储模式字节
        psu_data['pkt04']['mode_byte'] = payload[1]
        
        # 大端序解包 - 从偏移2开始，最多13个 uint16
        available = len(payload) - 2
        num_shorts = min(available // 2, 13)
        
        if num_shorts > 0:
            fmt = f'>{num_shorts}H'  # 大端序！
            v = struct.unpack(fmt, payload[2:2 + num_shorts * 2])
            
            # 存储原始值
            psu_data['pkt04']['raw_values'] = list(v)
            
            # 解析关键字段
            if num_shorts >= 1:
                # v00: 主温度 (High)
                psu_data['pkt04']['temp_main'] = v[0] / 10.0
            
            if num_shorts >= 2:
                # v01: 风扇PWM/周期
                psu_data['pkt04']['fan_pwm'] = v[1]
            
            if num_shorts >= 3:
                # v02: 负载状态 (356=低, 612=高)
                psu_data['pkt04']['load_status'] = v[2]
            
            if num_shorts >= 6:
                # v05: 辅助温度 (Low/MOS)
                psu_data['pkt04']['temp_secondary'] = v[5] / 10.0
            
            if num_shorts >= 8:
                # v07: AC输入功率 ***关键字段***
                psu_data['pkt04']['ac_power'] = v[6] / 10.0
                
                # 同步到pkt02用于效率计算
                psu_data['pkt02']['ac_power'] = v[6] / 10.0
                if psu_data['pkt02'].get('power_dc_total', 0) > 0 and v[6] > 0:
                    psu_data['pkt02']['efficiency'] = (psu_data['pkt02']['power_dc_total'] / (v[6] / 10.0)) * 100
                else:
                    psu_data['pkt02']['efficiency'] = 0
            
            if num_shorts >= 12:
                # v11: 气流温度 (x100)
                psu_data['pkt04']['temp_air'] = v[11] / 100.0
            
            # 解析风扇模式名称
            mode_names = {
                3: 'High/Custom',
                4: 'Auto',
                5: 'Mute',
                6: 'OC',
                7: 'Unknown',
                8: 'Clean'
            }
            psu_data['pkt04']['mode_name'] = mode_names.get(
                psu_data['pkt04']['mode_byte'], 
                f'Mode_{psu_data["pkt04"]["mode_byte"]}'
            )
        
        return True
        
    except Exception as e:
        print(f"[ERROR] 解析 0x04: {e}", file=sys.stderr)
        return False

def parse_packet_03_05(pkt_type, payload):
    """解析型号 (0x03) 和序列号 (0x05)"""
    try:
        # 去掉类型字节和尾部校验，提取 ASCII
        text = payload[1:-1].decode('ascii', errors='ignore').strip('\x00').strip()
        if pkt_type == 0x03:
            psu_data['model'] = text
            print(f"[INFO] 型号: {text}", file=sys.stderr)
        elif pkt_type == 0x05:
            psu_data['serial'] = text
            print(f"[INFO] 序列号: {text}", file=sys.stderr)
    except:
        pass

def get_tsv_header():
    """生成 TSV 表头"""
    headers = [
        'timestamp',
        'model', 'serial',
        
        # 0x02 解析值
        'volt_3v3', 'volt_5v', 'volt_12v', 'volt_5vsb',
        'curr_3v3', 'curr_5v', 'curr_12v',
        'power_3v3', 'power_5v', 'power_12v', 'power_dc_total',
        'ac_voltage', 'ac_power', 'ac_freq',
        'fan_rpm', 'efficiency',
        
        # 0x04 解析值
        'fan_mode', 'mode_name',
        'temp_main', 'temp_secondary', 'temp_air',
        'load_status', 'fan_pwm',
        
        # 0x02 原始值 (13个 uint16)
        'pkt02_v00', 'pkt02_v01', 'pkt02_v02', 'pkt02_v03', 'pkt02_v04',
        'pkt02_v05', 'pkt02_v06', 'pkt02_v07', 'pkt02_v08', 'pkt02_v09',
        'pkt02_v10', 'pkt02_v11', 'pkt02_v12',
        
        # 0x04 原始值 (mode + 13个值)
        'pkt04_mode',
        'pkt04_v00', 'pkt04_v01', 'pkt04_v02', 'pkt04_v03', 'pkt04_v04',
        'pkt04_v05', 'pkt04_v06', 'pkt04_v07', 'pkt04_v08', 'pkt04_v09',
        'pkt04_v10', 'pkt04_v11', 'pkt04_v12',
        
        # 原始 HEX
        'pkt02_raw_hex', 'pkt04_raw_hex',
    ]
    return '\t'.join(headers)

def get_tsv_row():
    """生成一行 TSV 数据"""
    def get(d, key, default=''):
        return str(d.get(key, default)) if d.get(key, default) != '' else ''
    
    def get_raw(d, idx, default=''):
        vals = d.get('raw_values', [])
        return str(vals[idx]) if idx < len(vals) else default
    
    row = [
        datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
        psu_data.get('model', ''),
        psu_data.get('serial', ''),
        
        # 0x02 解析值
        get(psu_data['pkt02'], 'volt_3v3'),
        get(psu_data['pkt02'], 'volt_5v'),
        get(psu_data['pkt02'], 'volt_12v'),
        get(psu_data['pkt02'], 'volt_5vsb'),
        get(psu_data['pkt02'], 'curr_3v3'),
        get(psu_data['pkt02'], 'curr_5v'),
        get(psu_data['pkt02'], 'curr_12v'),
        get(psu_data['pkt02'], 'power_3v3'),
        get(psu_data['pkt02'], 'power_5v'),
        get(psu_data['pkt02'], 'power_12v'),
        get(psu_data['pkt02'], 'power_dc_total'),
        get(psu_data['pkt02'], 'ac_voltage'),
        get(psu_data['pkt02'], 'ac_power'),
        get(psu_data['pkt02'], 'ac_freq'),
        get(psu_data['pkt02'], 'fan_rpm'),
        get(psu_data['pkt02'], 'efficiency'),
        
        # 0x04 解析值
        get(psu_data['pkt04'], 'mode_byte'),
        get(psu_data['pkt04'], 'mode_name'),
        get(psu_data['pkt04'], 'temp_main'),
        get(psu_data['pkt04'], 'temp_secondary'),
        get(psu_data['pkt04'], 'temp_air'),
        get(psu_data['pkt04'], 'load_status'),
        get(psu_data['pkt04'], 'fan_pwm'),
        
        # 0x02 原始值
        get_raw(psu_data['pkt02'], 0), get_raw(psu_data['pkt02'], 1),
        get_raw(psu_data['pkt02'], 2), get_raw(psu_data['pkt02'], 3),
        get_raw(psu_data['pkt02'], 4), get_raw(psu_data['pkt02'], 5),
        get_raw(psu_data['pkt02'], 6), get_raw(psu_data['pkt02'], 7),
        get_raw(psu_data['pkt02'], 8), get_raw(psu_data['pkt02'], 9),
        get_raw(psu_data['pkt02'], 10), get_raw(psu_data['pkt02'], 11),
        get_raw(psu_data['pkt02'], 12),
        
        # 0x04 原始值
        get(psu_data['pkt04'], 'mode_byte'),
        get_raw(psu_data['pkt04'], 0), get_raw(psu_data['pkt04'], 1),
        get_raw(psu_data['pkt04'], 2), get_raw(psu_data['pkt04'], 3),
        get_raw(psu_data['pkt04'], 4), get_raw(psu_data['pkt04'], 5),
        get_raw(psu_data['pkt04'], 6), get_raw(psu_data['pkt04'], 7),
        get_raw(psu_data['pkt04'], 8), get_raw(psu_data['pkt04'], 9),
        get_raw(psu_data['pkt04'], 10), get_raw(psu_data['pkt04'], 11),
        get_raw(psu_data['pkt04'], 12),
        
        # 原始 HEX
        get(psu_data['pkt02'], 'raw_hex'),
        get(psu_data['pkt04'], 'raw_hex'),
    ]
    
    return '\t'.join(row)

def read_packet_continuous(ser):
    """持续读取并解析数据包（非阻塞）"""
    packets_received = []
    
    while ser.in_waiting > 0:
        byte1 = ser.read(1)
        if byte1 == b'\x55':
            byte2 = ser.read(1)
            if byte2 == b'\x7E':
                len_byte = ser.read(1)
                if not len_byte:
                    continue
                
                pkt_len = ord(len_byte)
                body = ser.read(pkt_len)
                
                if len(body) == pkt_len:
                    # 去掉校验和尾部
                    payload = body[:-2] if len(body) >= 2 else body
                    
                    if len(payload) > 0:
                        pkt_type = payload[0]
                        packets_received.append(pkt_type)
                        
                        if pkt_type == 0x02:
                            parse_packet_02(payload)
                        elif pkt_type == 0x04:
                            parse_packet_04(payload)
                        elif pkt_type == 0x03 or pkt_type == 0x05:
                            parse_packet_03_05(pkt_type, payload)
    
    return packets_received

def print_status(count, packets_count):
    """打印当前状态到控制台"""
    p02 = psu_data['pkt02']
    p04 = psu_data['pkt04']
    
    if not p02:
        return
    
    # 使用正确的温度和功率值
    temp_main = p04.get('temp_main', 0)
    temp_sec = p04.get('temp_secondary', 0)
    temp_air = p04.get('temp_air', 0)
    mode_name = p04.get('mode_name', 'Unknown')
    
    dc = p02.get('power_dc_total', 0)
    ac = p02.get('ac_power', 0)  # 现在从pkt04同步过来的
    eff = p02.get('efficiency', 0)
    fan = p02.get('fan_rpm', 0)
    
    line = (f"\r[{count:>5}] Pkts:{packets_count:>6} | "
            f"AC: {p02.get('ac_voltage', 0):>5.1f}V {ac:>6.1f}W | "
            f"DC: {dc:>6.2f}W | "
            f"Eff: {eff:>5.1f}% | "
            f"Temp: {temp_main:>4.1f}/{temp_sec:>4.1f}/{temp_air:>4.1f}°C | "
            f"Fan: {fan:>4} RPM ({mode_name:>10})")
    
    print(line + ' ' * 10, end='', file=sys.stderr)

def main():
    # 打开串口
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)  # 很短的超时，非阻塞读取
        print(f"[INFO] 被动监听模式: {SERIAL_PORT} @ {BAUD_RATE}", file=sys.stderr)
        print(f"[INFO] 等待数据包...", file=sys.stderr)
    except Exception as e:
        print(f"[ERROR] 串口错误: {e}", file=sys.stderr)
        sys.exit(1)
    
    # 打开日志文件
    file_exists = os.path.exists(LOG_FILE)
    log_f = open(LOG_FILE, 'a', buffering=1)
    
    if not file_exists:
        log_f.write(get_tsv_header() + '\n')
        print(f"[INFO] 创建日志: {LOG_FILE}", file=sys.stderr)
    else:
        print(f"[INFO] 追加到: {LOG_FILE}", file=sys.stderr)
    
    print(f"[INFO] 开始被动监听, Ctrl+C 停止", file=sys.stderr)
    print(f"[INFO] 温度显示: 主/辅助/气流", file=sys.stderr)
    print("-" * 100, file=sys.stderr)
    
    try:
        count = 0
        total_packets = 0
        last_log_time = 0
        
        while True:
            # 持续读取数据包
            packets = read_packet_continuous(ser)
            
            if packets:
                total_packets += len(packets)
                
                # 根据设定的间隔记录数据
                current_time = time.time()
                if current_time - last_log_time >= LOG_INTERVAL:
                    # 只在有新数据时记录
                    if psu_data['pkt02'].get('last_update', 0) > last_log_time:
                        log_f.write(get_tsv_row() + '\n')
                        count += 1
                        last_log_time = current_time
                        
                        # 控制台输出
                        print_status(count, total_packets)
            
            # 短暂延时避免CPU占用过高
            time.sleep(0.01)
                
    except KeyboardInterrupt:
        print(f"\n[INFO] 停止，共记录 {count} 条，接收 {total_packets} 个数据包", file=sys.stderr)
    finally:
        log_f.close()
        ser.close()
        print(f"[INFO] 已保存: {LOG_FILE}", file=sys.stderr)

if __name__ == "__main__":
    main()
