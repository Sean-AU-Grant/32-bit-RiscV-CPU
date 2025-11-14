import binascii
import csv

compare_file = 'micro_3/sim_results.csv'
file_path = 'micro_3/mem_test.hex'
test_script = 'micro_3/micro_insts.csv'


def parse_file():
    all_instructions = []
    with open(file_path, 'r') as f:
        lines = f.readlines()
       
    for line in lines:
        try:
            
            # each line goes as follows:
            # [31:0]rs1, [31:0]imm, [31:0]store_data, [2:0]funct3, ld_st, [5:0] id, [5:0] addr_ptr,
            # 112 total bits per instruction
            
            byte_data = binascii.unhexlify(line.strip())
            
            bin_val = int.from_bytes(byte_data, byteorder='big')
            
            new_inst = int((bin_val >> 112) & 0x1)
            rs1 = int((bin_val >> 80) & 0xFFFFFFFF)
            imm = int((bin_val >> 48) & 0xFFFFFFFF)
            store_data = int((bin_val >> 16) & 0xFFFFFFFF)
            funct3 = int((bin_val >>13) & 0x3)
            ld_st = int((bin_val >> 12) & 0x1)
            id = int((bin_val >> 6) & 0x3F)
            addr = int((bin_val) & 0x3F)
            
            
            inst_record = {
                "new_inst": new_inst,
                "rs1": rs1,
                "imm": imm,
                "store_data": store_data,
                "funct3": funct3,
                "ld_st": ld_st,
                "id": id,
                "addr": addr
            
            }
            
            all_instructions.append(inst_record)
        except binascii.Error as e:
            print(f"Error converting hex string to bytes: {e}")

    return all_instructions

def sign_extend(value, bits):
    sign_bit_mask = 1 << (bits - 1)
    
    if(value & sign_bit_mask) != 0:
        extension_mask = (0xFFFFFFFF << bits) & 0xFFFFFFFF
        return (value | extension_mask)
    else:
        return value & 0xFFFFFFFF
    
def create_test_hex():
    read_data = []
    with open(test_script, 'r') as f:
        reader = csv.reader(f)
        
        for row in reader:
            read_data.append(row)
        
    output_hex = []
    for row in read_data:
        if not row[0].strip().isdigit(): 
            continue
        try:
            new_inst       = int(row[0].strip())
            rs1_val        = int(row[1].strip())
            imm_val        = int(row[2].strip())
            store_data_val = int(row[3].strip())
            funct3_val     = int(row[4].strip())
            ld_st_val      = int(row[5].strip())
            id_val         = int(row[6].strip())
            addr_val       = int(row[7].strip())
            
            row_int =   (new_inst       << 112) + \
                        (rs1_val        << 80)  + \
                        (imm_val        << 48)  + \
                        (store_data_val << 16)  + \
                        (funct3_val     << 13)  + \
                        (ld_st_val      << 12)  + \
                        (id_val         << 6)   + \
                        addr_val
        
            hex_string = f"{row_int:030X}"
        
            
            output_hex.append(hex_string)
            
        except ValueError as e:
            print(f"Warning: Could not convert a value to integer in row {row}: {e}")
            continue
        

        
    
    with open(file_path, 'w') as f:
        for lines in output_hex:
            f.write(lines + '\n')
  
def process_results(golden_results):
    rtl_results = []
    
    try:
        with open(compare_file, 'r') as f:
            reader = csv.reader(f)
            
            for rows in reader:
                if len(rows) < 3:
                    print(f"Warning: Skipping malformed line with content: {rows}")
                    continue
                
                rtl_results.append(rows)
                
    except FileNotFoundError:
        print(f"Error: RTL results file '{compare_file}' not found.")
        return
    

    
    for g_result in golden_results:
        g_result = g_result.split(",")
        found = False
        for item in rtl_results:
            if int(item[0],16) == int(g_result[0]) and int(item[1],16) == int(g_result[1]):
                if found:
                    print(f"Duplicate entry: id {g_result[0]}, addr {g_result[1]}")
                found = True
                # print("id found")
                if int(item[2], 16) == int(g_result[2]):
                    
                    print(f"Pass on id: {g_result[0]} and addr: {g_result[1]}")
                else:
                    print(f"Val Mismatch ID: {g_result[0]} addr: {g_result[1]} Expected: {g_result[2]}, Got: {int(item[2], 16)}")
                
                continue
        if(found == False):
            print(f"id not found: {g_result[0]} addr: {g_result[1]}")
            
        
        

def main():
    create_test_hex()
    mem_size_bytes = 64000000 # 64MB
    mem = [0] * mem_size_bytes
    golden_results = []
    insts = parse_file()
    # data_out = 0
    for inst in insts:
        if inst["new_inst"]:
            mem_addr = inst["rs1"] + inst["imm"]
            if inst["ld_st"]: #load
                
                match inst["funct3"]:
                    case 0: # store byte
                        data_out = sign_extend(mem[mem_addr], 8) 
                        # data_out = mem[mem_addr] & 0xFF
                    case 1: # store half
                        data_out = sign_extend((mem[mem_addr]<<8) + mem[mem_addr], 16)
                        # data_out = (mem[mem_addr+1]<<8) | mem[mem_addr]
                    case 2: # store word
                        data_out = (mem[mem_addr+3]<<24) | (mem[mem_addr+2]<<16) | (mem[mem_addr+1]<<8) | mem[mem_addr]
                    case 4:
                        data_out = mem[mem_addr] & 0xFF
                    case 5:
                        data_out = (mem[mem_addr+1]<<8) + mem[mem_addr]
                        
                golden_results.append(f"{inst['id']},{inst['addr']},{data_out}")
            else:   #store
                match inst["funct3"]:
                    case 0: # store byte
                        mem[mem_addr] = inst["store_data"] & 0xFF

                    case 1: # store half
                        mem[mem_addr] = inst["store_data"] & 0xFF
                        mem[mem_addr + 1] = (inst["store_data"] >> 8) & 0xFF


                    case 2: # store word
                        mem[mem_addr] = inst["store_data"] & 0xFF
                        mem[mem_addr + 1] = (inst["store_data"] >> 8) & 0xFF
                        mem[mem_addr + 2] = (inst["store_data"] >> 16) & 0xFF
                        mem[mem_addr + 3] = (inst["store_data"] >> 24) & 0xFF
    
    process_results(golden_results)



if __name__ == "__main__":
    main()