import os
import re
import datetime
from collections import defaultdict
from html import escape

def load_description_map(txt_file_path):
    desc_map = {}
    with open(txt_file_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.lower().startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) < 5:
                continue
            tc_id = parts[0].strip()
            description = parts[1].strip()
            sid = parts[2].strip().replace("0x", "").upper()
            sub = parts[3].strip().replace("0x", "").upper()
            positive_response = parts[4].strip().replace("0x", "").upper()
            key = (sid, sub)
            value = (description, tc_id, positive_response)
            if key not in desc_map:
                desc_map[key] = []
            desc_map[key].append(value)
    return desc_map

def parse_data_bytes(line):
    match = re.search(r'd\s+\d+\s+((?:[0-9A-Fa-f]{2}\s+)+)', line)
    if match:
        return match.group(1).strip().split()
    return []

def get_description(data_bytes):
    if not data_bytes or len(data_bytes) < 2:
        return "", "", ""
    sid_index = 2 if data_bytes[0].startswith("1") else 1
    if len(data_bytes) <= sid_index:
        return "", "", ""
    sid = data_bytes[sid_index].upper()
    for length in (3, 2, 1):
        if sid_index + length < len(data_bytes):
            sub = ''.join(data_bytes[sid_index + 1: sid_index + 1 + length]).upper()
            key = (sid, sub)
            if key in DESCRIPTION_MAP:
                used = getattr(get_description, "used_tc_ids", set())
                for desc, tc_id, expected_resp in DESCRIPTION_MAP[key]:
                    if tc_id not in used:
                        used.add(tc_id)
                        setattr(get_description, "used_tc_ids", used)
                        return desc, tc_id, expected_resp
                return DESCRIPTION_MAP[key][0]
    return "", "", ""

def get_failure_reason(nrc):
    reasons = {
        "10" : "generalReject",
        "11" : "serviceNotSupported",
        "12" : "subFunctionNotSupported",
        "13" : "incorrectMessageLengthOrInvalidFormat",
        "14" : "responseTooLong",
        "21" : "busyRepeatReques",
        "22" : "conditionsNotCorrect",
        "23" : "ISOSAEReserved",
        "24" : "requestSequenceError",
        "31" : "requestOutOfRange",
        "32" : "ISOSAEReserved",
        "33" : "securityAccessDenied",
        "34" : "ISOSAEReserved",
        "35" : "invalidKey",
        "36" : "exceedNumberOfAttempts",
        "37" : "requiredTimeDelayNotExpired",
        "70" : "uploadDownloadNotAccepted",
        "71" : "transferDataSuspended",
        "72" : "generalProgrammingFailure",
        "73" : "wrongBlockSequenceCounter",
        "78" : "requestCorrectlyReceived-ResponsePending",
        "7E" : "subFunctionNotSupportedInActiveSession",
        "7F" : "serviceNotSupportedInActiveSession",
        "80" : "ISOSAEReserved",
        "81" : "rpmTooHigh",
        "82" : "rpmTooLow",
        "83" : "engineIsRunning",
        "84" : "engineIsNotRunning",
        "85" : "engineRunTimeTooLow",
        "86" : "temperatureTooHigh",
        "87" : "temperatureTooLow",
        "88" : "vehicleSpeedTooHigh",
        "89" : "vehicleSpeedTooLow",
        "8A" : "throttle/PedalTooHigh",
        "8B" : "throttle/PedalTooLow",
        "8C" : "transmissionRangeNotInNeutral",
        "8D" : "transmissionRangeNotInGear",
        "8E" : "ISOSAEReserved",
        "8F" : "brakeSwitch(es)NotClosed (Brake Pedal not pressed or not applied)",
        "90" : "shifterLeverNotInPark",
        "91" : "torqueConverterClutchLocked",
        "92" : "voltageTooHigh",
        "93" : "voltageTooLow",
        "FF" : "ISOSAEReserved",
    }
    return reasons.get(nrc.upper(), f"Unknown NRC: {nrc}")

def get_status(data_bytes, expected_resp):
    if not data_bytes or len(data_bytes) < 3:
        return "Fail", "Incomplete response"

    if data_bytes[0].upper() == "10":
        actual_sid = data_bytes[2].upper()
    else:
        actual_sid = data_bytes[1].upper()

    if actual_sid == "7F":
        if len(data_bytes) >= 4 and data_bytes[3].upper() == "78":
            return "Pending", ""
        nrc = data_bytes[3].upper()
        if nrc == expected_resp:
            return "Pass", ""
        else:
            return "Fail", get_failure_reason(nrc)

    if actual_sid == expected_resp:
        return "Pass", ""

    return "Fail", f"Unexpected response: {actual_sid}"

def parse_line(line):
    line = line.strip()
    if not line or " d " not in line:
        return None
    parts = line.split()
    try:
        timestamp = float(parts[0])
    except:
        return None
    can_id = parts[2].upper()
    direction = parts[3]
    data_bytes = parse_data_bytes(line)
    return {
        "timestamp": timestamp,
        "can_id": can_id,
        "direction": direction,
        "data_bytes": data_bytes,
        "raw": line
    }



def parse_asc_file(asc_file_path, allowed_tx_ids, allowed_rx_ids):
    messages_by_tc = defaultdict(list)
    current_request = None
    start_ts, end_ts = None, None
    skip_next_fc = False
    pending_flag = False
    rx_multi_response_pending = False
    rx_multi_response_first = None

    allowed_tx_ids = set(f"{id:X}" for id in allowed_tx_ids)
    allowed_rx_ids = set(f"{id:X}" for id in allowed_rx_ids)

    with open(asc_file_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or not re.match(r"^\d+\.\d+", line):
                continue

            msg = parse_line(line)
            if not msg:
                continue

            can_id = msg["can_id"].upper()
            direction = msg["direction"]
            data_bytes = msg["data_bytes"]

            if direction == "Tx" and can_id in allowed_tx_ids:
                if data_bytes and data_bytes[0].upper() == "10":
                    skip_next_fc = True
                desc, tc_id, expected_resp = get_description(data_bytes)
                if desc and tc_id:
                    current_request = {
                        "timestamp": msg["timestamp"],
                        "can_id": can_id,
                        "direction": direction,
                        "data_bytes": data_bytes,
                        "desc": desc,
                        "tc_id": tc_id,
                        "expected_resp": expected_resp,
                        "status": "Pending"
                    }

            elif direction == "Rx" and can_id in allowed_rx_ids:
                if skip_next_fc and data_bytes and data_bytes[0].upper() == "30":
                    skip_next_fc = False
                    continue

                # Handle response pending (0x7F .. 78)
                if data_bytes and len(data_bytes) >= 4 and data_bytes[1].upper() == "7F" and data_bytes[3].upper() == "78":
                    pending_flag = True
                    continue

                # Handle multi-frame Rx: 0x10 + 0x21
                if data_bytes and data_bytes[0].upper() == "10":
                    rx_multi_response_first = msg
                    rx_multi_response_pending = True
                    continue

                if rx_multi_response_pending and data_bytes and data_bytes[0].upper() == "21":
                    combined_bytes = rx_multi_response_first["data_bytes"][:7] + data_bytes[1:]
                    rx_msg = {
                        "timestamp": rx_multi_response_first["timestamp"],
                        "can_id": rx_multi_response_first["can_id"],
                        "direction": rx_multi_response_first["direction"],
                        "data_bytes": combined_bytes
                    }
                    if current_request:
                        status, reason = get_status(combined_bytes, current_request.get("expected_resp", ""))
                        current_request.update({
                            "response": rx_msg,
                            "status": status,
                            "failure_reason": reason
                        })
                        messages_by_tc[current_request["tc_id"]].append(current_request)

                        req_ts = current_request["timestamp"]
                        res_ts = rx_msg["timestamp"]
                        start_ts = min(start_ts or req_ts, req_ts)
                        end_ts = max(end_ts or res_ts, res_ts)

                        current_request = None

                    rx_multi_response_pending = False
                    rx_multi_response_first = None
                    continue

                # If response was pending, now handle actual response
                if pending_flag:
                    if current_request:
                        status, reason = get_status(data_bytes, current_request.get("expected_resp", ""))
                        current_request.update({
                            "response": msg,
                            "status": status,
                            "failure_reason": reason
                        })
                        messages_by_tc[current_request["tc_id"]].append(current_request)

                        req_ts = current_request["timestamp"]
                        res_ts = msg["timestamp"]
                        start_ts = min(start_ts or req_ts, req_ts)
                        end_ts = max(end_ts or res_ts, res_ts)

                        current_request = None
                    pending_flag = False
                    continue

                # Regular Rx response
                if current_request:
                    status, reason = get_status(data_bytes, current_request.get("expected_resp", ""))
                    current_request.update({
                        "response": msg,
                        "status": status,
                        "failure_reason": reason
                    })
                    messages_by_tc[current_request["tc_id"]].append(current_request)

                    req_ts = current_request["timestamp"]
                    res_ts = msg["timestamp"]
                    start_ts = min(start_ts or req_ts, req_ts)
                    end_ts = max(end_ts or res_ts, res_ts)

                    current_request = None

    return messages_by_tc, start_ts or 0, end_ts or 0




def generate_html_report(messages_by_tc, output_path, asc_filename, start_ts, end_ts, ecu_info_data=None, target_ecu=None):
    total = len(messages_by_tc)
    passed = sum(1 for tc in messages_by_tc.values() if all(msg["status"] == "Pass" for msg in tc))
    failed = total - passed
    duration = end_ts - start_ts
    generated_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    html = f"""<!DOCTYPE html>
<html>
<head><title>UDS Diagnostic Report</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<style>
  body {{ font-family: Arial; margin: 20px; }}
  .pass {{ color: green; font-weight: bold; }}
  .fail {{ color: red; font-weight: bold; }}
  .wrapper {{
    display: flex;
    justify-content: center;
    align-items: flex-start;
    gap: 50px;
    margin-top: 20px;
  }}
  .summary-block {{ text-align: left; min-width: 250px; }}
  #chart-container {{ width: 300px; }}
  table {{ border-collapse: collapse; width: 100%; margin-top: 10px; }}
  th, td {{ border: 1px solid #ccc; padding: 8px; }}
  th {{ background: #f0f0f0; }}
  summary {{ font-weight: bold; cursor: pointer; }}
</style>
</head>
<body>

<h1 style="text-align: center;">UDS Diagnostic Report</h1>

<div style="display: flex; justify-content: flex-start; align-items: flex-start; gap: 40px; margin-top: 20px; padding-left: 10px;">
    <div style="width: 650px;">
    
        {f"<p><strong>Target ECU:</strong> {escape(target_ecu)}</p>" if target_ecu else ""}
        {"".join(f"<p><strong>{escape(k)}:</strong> {escape(v)}</p>" for k, v in ecu_info_data.items()) if ecu_info_data else ""}
        
        <hr style="width: 300px;border:1px solid #999; margin:25px 0;">
        
        <p><strong>Generated:</strong> {generated_time}</p>
        <p><strong>CAN Log File:</strong> {asc_filename}</p>
        <p><strong>Total Test Cases:</strong> {total}</p>
        <p class="pass"><strong>Passed:</strong> {passed}</p>
        <p class="fail"><strong>Failed:</strong> {failed}</p>
        <p><strong>Test Duration:</strong> {duration:.3f} seconds</p>
        
    </div>

    <div id="chart-container" style="width: 320px; margin-left:70px;">
        <canvas id="passFailChart" width="300" height="300"></canvas>
    </div>
</div>


<script>
  const ctx = document.getElementById('passFailChart').getContext('2d');
  new Chart(ctx, {{
    type: 'pie',
    data: {{
      labels: ['Passed', 'Failed'],
      datasets: [{{
        data: [{passed}, {failed}],
        backgroundColor: ['#4CAF50', '#F44336']
      }}]
    }},
    options: {{
      responsive: true,
      plugins: {{
        legend: {{ position: 'bottom' }},
        title: {{ display: true, text: 'Test Case Results' }}
      }}
    }}
  }});
</script>

<hr><br>
"""

    for tc_id, steps in messages_by_tc.items():
        status = steps[0]['status']
        status_class = 'pass' if status == 'Pass' else 'fail'
        html += f"<details><summary>{tc_id} - <span class='{status_class}'>{status}</span></summary>\n"
        html += """<table><tr><th>Step</th><th>Description</th><th>Timestamp</th><th>Type</th><th>Status</th><th>Failure Reason</th></tr>\n"""
        
        step_count = 1
        for msg in steps:
            desc = msg['desc']
            combined_desc = ""
	    
            # Case 1: PreCondition and Testcase
            if "PreCondition:" in desc and "Testcase" in desc:
                parts = desc.split("PreCondition:", 1)[1].split("Testcase", 1)
                pre_detail = parts[0].strip()
                tc_detail = parts[1].strip()
                combined_desc = f"<b>PreCondition:</b> {escape(pre_detail)}<br><b>Testcase:</b>{escape(tc_detail)}"
	    
            # Case 2: Only PreCondition
            elif "PreCondition:" in desc:
                pre_detail = desc.split("PreCondition:", 1)[1].strip()
                combined_desc = f"<b>PreCondition:</b> {escape(pre_detail)}"
	    
            # Case 3: Only Testcase or any other
            else:
                combined_desc = escape(desc.strip())
	    
            # Request row
            html += f"<tr><td>{step_count}</td><td>{combined_desc}</td><td>{msg['timestamp']:.6f}</td><td>Request Sent</td><td></td><td>-</td></tr>\n"
            step_count += 1
	    
            # Response row
            response = msg.get("response", {})
            html += f"<tr><td>{step_count}</td><td></td><td>{response.get('timestamp', ''):.6f}</td><td>Response Received</td><td>{msg['status']}</td><td>{msg.get('failure_reason', '')}</td></tr>\n"
            step_count += 1
	    
        html += "</table></details>\n"
        
    html += "</body></html>"
    
    with open(output_path, "w", encoding="utf-8") as f:
         f.write(html)
    
    print(f"UDS HTML Report generated at:\n{output_path}\n")

def generate_report(asc_file_path, txt_file_path, output_html_file, allowed_tx_ids, allowed_rx_ids, ecu_info_data=None, target_ecu=None):
    global DESCRIPTION_MAP
    DESCRIPTION_MAP = load_description_map(txt_file_path)
    get_description.used_tc_ids = set()

    messages_by_tc, start_ts, end_ts = parse_asc_file(
        asc_file_path, allowed_tx_ids, allowed_rx_ids
    )

    report_path = output_html_file

    generate_html_report(
        messages_by_tc,
        report_path,
        os.path.basename(asc_file_path),
        start_ts,
        end_ts,
        ecu_info_data,
        target_ecu
    )
