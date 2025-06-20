import can
import socket
import shutil 
import os
import isotp
import time
import logging
from datetime import datetime
from udsoncan.client import Client
from udsoncan.connections import PythonIsoTpConnection
from udsoncan.configs import default_client_config
from drivers.Parse_handler import load_testcases
from drivers.can_logger import CANLogger
from udsoncan import AsciiCodec
from drivers.report_generator import generate_report 
from udsoncan.services import WriteDataByIdentifier

class SafeAsciiCodec(AsciiCodec):
    def decode(self, data):
        try:
            return data.decode('ascii')
        except UnicodeDecodeError:
            return data.hex()


class UDSClient:
    def __init__(self, config):
        can_cfg = config["uds"]["can"]
        isotp_cfg = config["uds"]["isotp"]
        timing_cfg = config["uds"]["timing"]
        self.uds_config = config["uds"]

        self.target_ecu = config["uds"].get("target_ecu", "Unknown ECU")
        self.context = {}

       
        self.info_dids = self.uds_config.get("ecu_information_dids", {})
        self.decode_dids = self.uds_config.get("decoding_dids", {})
        self.write_data_dict = self.uds_config.get("write_data", {})
        self.step_delays=self.uds_config.get("delays",{})
        self.default_delay=self.step_delays.get("default",0.5)

       
        self.client_config = default_client_config.copy()
        self.client_config["p2_timeout"] = timing_cfg["p2_client"] / 1000.0
        self.client_config["p2_star_timeout"] = timing_cfg["p2_extended_client"] / 1000.0
        self.client_config["s3_client_timeout"] = timing_cfg["s3_client"] / 1000.0
        self.client_config["exception_on_negative_response"] = False
        self.client_config["exception_on_unexpected_response"] = False
        self.client_config["exception_on_invalid_response"] = False
        self.client_config["use_server_timing"] = False

        
        self.client_config["data_identifiers"] = {
            int(did_str, 16): SafeAsciiCodec(length)
            for did_str, length in self.decode_dids.items()
        }
        self.client_config["write_data"] = {
            int(did_str, 16): data_str
            for did_str, data_str in self.write_data_dict.items()
        }

        
        addr_modes_cfg = self.uds_config["addressing_modes"]
        self.physical_conn = self._create_connection(addr_modes_cfg.get("physical"), can_cfg, isotp_cfg, "physical")
        self.functional_conn = self._create_connection(addr_modes_cfg.get("functional"), can_cfg, isotp_cfg, "functional")

        self.active_conn = self.physical_conn
        self.active_mode = "physical"

        
        self.allowed_ids = list({
            int(addr_modes_cfg.get("physical", {}).get("tx_id", "0"), 16),
            int(addr_modes_cfg.get("physical", {}).get("rx_id", "0"), 16),
            int(addr_modes_cfg.get("functional", {}).get("tx_id", "0"), 16),
            int(addr_modes_cfg.get("functional", {}).get("rx_id", "0"), 16),
        })

        self.allowed_tx_ids = [
                  int(addr_modes_cfg.get("physical", {}).get("tx_id", "0"), 16),
                  int(addr_modes_cfg.get("functional", {}).get("tx_id", "0"), 16)
              ]

        self.allowed_rx_ids = [
             int(addr_modes_cfg.get("physical", {}).get("rx_id", "0"), 16),
             int(addr_modes_cfg.get("functional", {}).get("rx_id", "0"), 16)
         ]


        filters = self.get_can_filters()
        self.project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
        log_dir = os.path.join(self.project_root, 'output', 'can_logs')
        self.can_logger = CANLogger(channel=can_cfg["channel"], interface=can_cfg["interface"], log_dir=log_dir,filters=filters)


    def get_can_filters(self):
        filters_enabled = self.uds_config.get("logging", {}).get("filters", False)
        
        if not filters_enabled: 
            logging.info("CANLogger: Logging ALL CAN messages (no filters)")
            return None

        addr_modes_cfg = self.uds_config["addressing_modes"]
        tx_id_phys = int(addr_modes_cfg.get("physical", {}).get("tx_id", "0"), 16)
        rx_id_phys = int(addr_modes_cfg.get("physical", {}).get("rx_id", "0"), 16)

        tx_id_func = int(addr_modes_cfg.get("functional", {}).get("tx_id", "0"), 16)
        rx_id_func = int(addr_modes_cfg.get("functional", {}).get("rx_id", "0"), 16)

        filters = [
            {"can_id": tx_id_phys, "can_mask": 0x7FF, "extended": False},
            {"can_id": rx_id_phys, "can_mask": 0x7FF, "extended": False},
            {"can_id": tx_id_func, "can_mask": 0x7FF, "extended": False},
            {"can_id": rx_id_func, "can_mask": 0x7FF, "extended": False}
        ]

        logging.info("CANLogger: Logging only UDS traffic (tx/rx physical+functional)")
        return filters
    
    
    def _create_connection(self, addr_cfg, can_cfg, isotp_cfg, mode_name):
        if not addr_cfg:
            print(f"No config found for {mode_name} addressing, skipping.")
            return None

        tx_id = int(addr_cfg["tx_id"], 16)
        rx_id = int(addr_cfg["rx_id"], 16)
        is_extended = addr_cfg.get("is_extended", False)

        address = isotp.Address(
            addressing_mode=isotp.AddressingMode.Normal_29bits if is_extended else isotp.AddressingMode.Normal_11bits,
            txid=tx_id,
            rxid=rx_id
        )

        rx_mask = 0x1FFFFFFF if is_extended else 0x7FF
        bus = can.interface.Bus(
            channel=can_cfg["channel"],
            bustype=can_cfg["interface"],
            fd=can_cfg.get("can_fd", True),
            can_filters=[{
                "can_id": rx_id,
                "can_mask": rx_mask,
                "extended": is_extended
            }]
        )

        stack = isotp.CanStack(bus=bus, address=address, params=isotp_cfg)
        conn = PythonIsoTpConnection(stack)

        return {
            "conn": conn,
            "client_config": self.client_config,
            "mode_name": mode_name
        }
    def switch_mode(self, mode):
           mode = mode.lower()
           if mode == "physical" and self.physical_conn is not None:
               self.active_conn = self.physical_conn
               self.active_mode = "physical"
               #print("Switched to physical addressing")
           elif mode == "functional" and self.functional_conn is not None:
               self.active_conn = self.functional_conn
               self.active_mode = "functional"
               #print("Switched to functional addressing")
           else:
               raise ValueError(f"Unsupported or unconfigured addressing mode: {mode}")
    

    def check_disk_space(self, min_required_mb=50):
        total, used, free = shutil.disk_usage("/")
        free_mb = free // (1024 * 1024)  # Convert to MB
        return (free_mb >= min_required_mb, free_mb)

    def start_logging(self, log_name_suffix=""):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"CANLog_{log_name_suffix}_{timestamp}.asc"
        self.can_logger.start(filename=filename)

    def stop_logging(self):
        self.can_logger.stop()


                    
    def timestamp_log(self):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        return timestamp

    def check_memory(self, oled):
        min_required = 50
        enough_space, free_mb = self.check_disk_space(min_required_mb=min_required)
        if not enough_space:
            warning_msg = f"Low Storage!\nOnly {free_mb}MB left.\nNeed {min_required}MB."
            oled.display_centered_text(warning_msg)
            logging.warning(warning_msg)
            time.sleep(4)
            return False

        oled.display_centered_text(f"Storage OK\nFree: {free_mb} MB")
        logging.info(f"Storage check passed: {free_mb} MB available")
        time.sleep(2)
        return True

    def try_basic_communication(self):
        try:
            with Client(self.active_conn["conn"], request_timeout=2, config=self.client_config) as client:
                response = client.tester_present()
                return response.positive
        except Exception as e:
            logging.warning(f"Tester Present failed: {e}")
            return False

    def get_ecu_information(self, oled=None ,logging_enable=True):
     
        if logging_enable:
                self.start_logging(log_name_suffix="ECU_Info")
                
        ecu_info = {}
        session_default = int(self.uds_config["default_session"], 16)
        session_extended = int(self.uds_config["extended_session"], 16)
        grouped_cases = load_testcases()
        
        def normalize_hex_string(val):
            return val.lower().replace("0x", "").strip()
        
        with Client(self.active_conn["conn"], request_timeout=2, config=self.client_config) as client:
            try:
                client.change_session(session_default)
                time.sleep(0.2)
                client.change_session(session_extended)
                time.sleep(0.2)
            except Exception as e:
                if oled:
                    oled.display_centered_text(f"Session Error:\n{str(e)}")
                logging.error(f"Session change failed: {e}")
                
                return
        
            # Go through all RDxxx
            for tc_id, steps in grouped_cases.items():
                if not tc_id.startswith("RD"):
                    continue
        
                logging.info(f"[ECU Info] Processing {tc_id}")
        
                for step in steps:
                    logging.debug(f"[ECU Info] Step={step}")
        
                    try:
                        tc_id, step_desc, service, subfunc, expected, *rest = step
        
                        service_clean = normalize_hex_string(service)
                        subfunc_clean = normalize_hex_string(subfunc)
        
                        if service_clean != "22":
                            continue
        
                        try:
                            did = int(subfunc_clean, 16)
                        except ValueError as ve:
                            logging.error(f"[ECU Info] Invalid subfunc '{subfunc}' in {tc_id} step '{step_desc}': {ve}")
                            continue
        
                        response = client.read_data_by_identifier(did)
                        if response.positive:
                            values = response.service_data.values[did]
                            if isinstance(values, (bytes, bytearray)):
                                hex_str = ' '.join(f"{b:02X}" for b in values)
                            elif isinstance(values, str):
                                hex_str = values
                            else:
                                hex_str = str(values)
        
                            ecu_info[step_desc] = hex_str
        
                            if oled:
                                oled.display_centered_text(f"{step_desc}\n{hex_str}")
                                time.sleep(2)
        
                            logging.info(f"[ECU Info] {step_desc} ({subfunc}) = {hex_str}")
        
                        else:
                            nrc = hex(response.code)
                            ecu_info[step_desc] = f"NRC: {nrc}"
        
                            if oled:
                                oled.display_centered_text(f"{step_desc}\nNRC: {nrc}")
        
                            logging.warning(f"[ECU Info] {step_desc} - NRC: {nrc}")
        
                    except Exception as e:
                        error_msg = str(e)[:40]
                        ecu_info[step_desc] = f"Error: {error_msg}"
        
                        if oled:
                            oled.display_centered_text(f"{step_desc}\nError: {error_msg}")
        
                        logging.error(f"[ECU Info] {step_desc} - Exception: {e}")
        
                    time.sleep(0.1)
        
        
        
        if logging_enable:
                self.stop_logging()
        return ecu_info 
           

    def run_testcase(self, oled):
        if not self.check_memory(oled):
            return

        self.start_logging(log_name_suffix="Testcase")
        ecu_info_data = self.get_ecu_information(oled=None,logging_enable=False)
        grouped_cases = load_testcases()
        self.context = {}
        
        for tc_id, steps in grouped_cases.items():
                
            if not tc_id.startswith("TC"):
                    continue    
            
            print("\n")
            logging.info(f"Running Test Case: {tc_id}")
                            
            for step in steps:
                tc_id, step_desc, service, subfunc, expected, *rest = step

                # Get addressing mode from step or default to physical
                addressing = rest[0].strip().lower() if rest else "physical"
                try:
                    self.switch_mode(addressing)
                   
                    
                    with Client(self.active_conn["conn"], request_timeout=2, config=self.active_conn["client_config"]) as client:
                                
                        logging.info(f"Switched to {addressing} mode for TC: {tc_id} Step: {step_desc}")
                        service_int = int(service, 16)
                        #subfunc_int = int(subfunc, 16)
                        expected_bytes = [int(b, 16) for b in expected.strip().split()]
                        write_data_dict = self.active_conn["client_config"].get('write_data', {})
                  
                        logging.info(f"Loaded write data:{list(write_data_dict.keys())}")
                        
                        logging.info(f"{tc_id} - {step_desc}: SID={service}, Sub={subfunc}, Expected={expected_bytes}")

                        response=None
                        if service_int == 0x10:
                            try:
                                # Clean and validate subfunction
                                subfunc_clean = subfunc.strip().replace(" ", "")  # remove spaces
                                if subfunc_clean and not all(c in "0123456789abcdefABCDEF" for c in subfunc_clean):
                                    raise ValueError(f"Invalid hex in subfunction: {subfunc_clean}")

                                subfunc_bytes = bytes.fromhex(subfunc_clean) if subfunc_clean else b''

                                # Build and send raw request
                                raw_request = bytearray([service_int]) + subfunc_bytes
                                logging.info(f"{tc_id} - {step_desc}: Sending {raw_request.hex().upper()}")

                                client.conn.send(raw_request)
                                time.sleep(0.2)
                                response_data = client.conn.wait_frame(timeout=2)

                                if response_data:
                                    logging.info(f"{tc_id} - Received: {response_data.hex().upper()}")
                                    expected_bytes = [int(b, 16) for b in expected.strip().split()]
                                    if response_data[0] == expected_bytes[0]:
                                        logging.info(f"{tc_id} - {step_desc} -> PASS")
                                    else:
                                        logging.warning(f"{tc_id} - {step_desc} -> FAIL - Unexpected SID: {response_data[0]:02X}")
                                else:
                                    logging.warning(f"{tc_id} - No response received")

                            except ValueError as ve:
                                logging.error(f"{tc_id} - Hex Error: {str(ve)}")
                                oled.display_centered_text(f"{tc_id}\nHex Error")
                            except Exception as e:
                                logging.error(f"{tc_id} - Exception: {type(e).__name__} - {str(e)}")
                                oled.display_centered_text(f"{tc_id}\nError: {str(e)[:16]}")
                            finally:
                                oled.display_centered_text(f"{tc_id}\n{step_desc[:20]}")
                                time.sleep(2)

                        elif service_int == 0x11:  # ECU Reset
                            raw_request = bytes([0x11, subfunc_int])
                            client.conn.send(raw_request)
                            response = client.conn.wait_frame(timeout=2)
                        elif service_int == 0x22:  # ReadDataByIdentifier
                            did_hi = (subfunc_int >> 8) & 0xFF
                            did_lo = subfunc_int & 0xFF
                            raw_request = bytes([0x22, did_hi, did_lo])
                            client.conn.send(raw_request)
                            response = client.conn.wait_frame(timeout=2)
                        elif service_int == 0x2E:  # WriteDataByIdentifier
                            data_to_write = write_data_dict.get(subfunc_int)
                            if data_to_write is None:
                                raise ValueError(f"No write data configured for DID {hex(subfunc_int)}")
                            response = client.write_data_by_identifier(subfunc_int, data_to_write)
                        elif service_int == 0x19:  # ReadDTCInformation
                            # Status mask 0xFF is typically used, but you can customize this
                            status_mask = 0xFF
                            raw_request = bytes([0x19, subfunc_int, status_mask])
                            client.conn.send(raw_request)
                            response_data = client.conn.wait_frame(timeout=2)
                            
                        elif service_int == 0x14:  # ClearDiagnosticInformation
                            # 0x14 SID + 3-byte GroupOfDTC (e.g., FFFFFFFF)
                            dtc_group_bytes = [(subfunc_int >> shift) & 0xFF for shift in (16, 8, 0)]
                            raw_request = bytes([0x14] + dtc_group_bytes)
                            client.conn.send(raw_request)
                            response_data = client.conn.wait_frame(timeout=2)
                            
                        elif service_int == 0x3E:  # TesterPresent
                            raw_request = bytes([0x3E, 0x00])
                            client.conn.send(raw_request)
                            response_data = client.conn.wait_frame(timeout=2)
                            
                        elif service_int == 0x85:  # ControlDTCSetting
                             raw_request = bytes([0x85, subfunc_int])
                             client.conn.send(raw_request)
                             response_data = client.conn.wait_frame(timeout=2)
                        
                        elif service_int == 0x27:
                             if subfunc_int % 2 == 1:  
                                 response = client.request_seed(subfunc_int)
                                 if not response.positive:
                                     failure_reason = f"NRC (seed): {hex(response.code)}"
                                     logging.warning(f"{tc_id} {step_desc} -> FAIL - {failure_reason}")
                                     raise Exception(failure_reason)
                                 
                                 seed = response.service_data.seed
                                 self.context[f"seed_{subfunc_int}"] = seed
                                 logging.info(f"Received Seed (subfunc {hex(subfunc_int)}): {seed.hex()}")
                                 time.sleep(0.5)
                             
                                 # Send seed to PC and get key
                                 udp_ip = "192.168.10.220"
                                 udp_port = 5005
                                 max_retries = 3
                                 retry_delay = 1.0
                                 expected_key_length = 8  
                             
                                 sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                                 sock.settimeout(5)
                             
                                 try:
                                     for attempt in range(1, max_retries + 1):
                                         try:
                                             logging.info(f"Attempt {attempt}: Sending seed to PC...")
                                             sock.sendto(seed.hex().encode(), (udp_ip, udp_port))
                                             key, _ = sock.recvfrom(1024)
                                             key = key.strip()
                             
                                             if not key:
                                                 raise Exception("Received empty key from PC")
                                             if len(key) != expected_key_length:
                                               raise Exception(f"Invalid key length: expected {expected_key_length}, got {len(key)}")
                                             
                                             self.context[f"key_{subfunc_int+1}"] = key  # store key using subfunc 0x02/0x12
                                             logging.info(f"Received Key (for subfunc {hex(subfunc_int+1)}): {key}")
                                             break
                                         except socket.timeout:
                                             logging.warning(f"Attempt {attempt} - Timeout waiting for key.")
                                             if attempt < max_retries:
                                                 time.sleep(retry_delay)
                                             else:
                                                 raise Exception(f"Timeout after {max_retries} retries waiting for key from PC")
                                         except Exception as e:
                                             logging.exception(f"Attempt {attempt} - Error occurred:")
                                             if attempt == max_retries:
                                                 raise
                                 finally:
                                     sock.close()
                             
                             elif subfunc_int % 2 == 0:  
                                 key = self.context.get(f"key_{subfunc_int}")
                                 if not key:
                                     raise Exception(f"No key available for subfunction {hex(subfunc_int)}. Ensure seed request precedes key send.")
                                 
                                 response = client.send_key(subfunc_int, key)
                                 if not response.positive:
                                     failure_reason = f"NRC (key): {hex(response.code)}"
                                     logging.warning(f"{tc_id} {step_desc} -> FAIL - {failure_reason}")
                                     raise Exception(failure_reason)
                             else:
                                 raise ValueError(f"Unsupported subfunction for service 0x27: {hex(subfunc_int)}")
                              
                        elif service_int == 0x28:
                            communication_type = 0x00  # Default communication type if not provided separately
                            raw_request = bytes([0x28, subfunc_int, communication_type])
                            client.conn.send(raw_request)
                            response_data = client.conn.wait_frame(timeout=2)
                       
                        else:
                            raise ValueError(f"Unsupported service: {service}")                   

                        status = "Fail"
                        failure_reason = "-"
                        if response.positive:
                            actual = list(response.original_payload)
                            if actual[:len(expected_bytes)] == expected_bytes:
                                status = "Pass"
                                logging.info(f"{tc_id} {step_desc}-> PASS")
                            else:
                                failure_reason = f"Expected {expected_bytes}, got {actual}"
                                logging.warning(f"{tc_id} {step_desc} -> FAIL - {failure_reason}")
                        else:
                            failure_reason = f"NRC: {hex(response.code)}"
                            logging.warning(f"{tc_id} {step_desc} -> FAIL - {failure_reason}")
                except Exception as e:
                    status = "Fail"
                    failure_reason = str(e)
                    logging.error(f"{tc_id} {step_desc} -> EXCEPTION - {failure_reason}")

                delay_key=service.upper()
                delay=float(self.step_delays.get(delay_key,self.default_delay))
                oled.display_centered_text(f"{tc_id}\n{step_desc[:20]}\n{status}")
                time.sleep(delay)

        self.stop_logging()
        time.sleep(1.5)

        full_log_path = self.can_logger.get_log_path() or "N/A"
        can_log_file = os.path.basename(full_log_path)

        # Confirm log file presence
        if not os.path.isfile(full_log_path):
            logging.error(f"File not found after logging stopped: {full_log_path}")
            oled.display_centered_text("Log Error!\nFile Missing.")
            return
        else:
            
            oled.display_centered_text("Log Generated!\n")
            time.sleep(2)

        report_dir = os.path.join(self.project_root, 'output', 'html_reports')
        os.makedirs(report_dir, exist_ok=True)
        report_filename = f"UDS_Report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.html"
        report_path = os.path.join(report_dir, report_filename)
        testcase_file_path="/home/mobase/Pld/udsoncan/input/supportFiles/testcase.txt"
        # Wait for log file to appear (max 3 seconds)
        for _ in range(6):
            if os.path.exists(full_log_path):
                print(f"Log file found: {full_log_path}")
                break
            else:
                print(f" Waiting for log file to appear: {full_log_path}")
                time.sleep(0.5)
        else:
            print(f"File not found: {can_log_file}")

        generate_report(
            asc_file_path=full_log_path,
            txt_file_path=testcase_file_path,
            output_html_file=report_path,
            allowed_tx_ids=self.allowed_tx_ids,
            allowed_rx_ids =self.allowed_rx_ids,
            ecu_info_data = ecu_info_data,
            target_ecu=self.target_ecu
        )

        oled.display_centered_text("Report Generated")
        time.sleep(2)
       


