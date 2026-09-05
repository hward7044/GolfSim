import usb.core
import sys

def parse_descriptors():
    # Vendor=0x0c45, Product=0x6366
    dev = usb.core.find(idVendor=0x0c45, idProduct=0x6366)
    if dev is None:
        print("Arducam camera not found by pyusb.")
        sys.exit(1)
        
    try:
        # Get configuration descriptor (Type 2, Index 0)
        cfg_desc = dev.ctrl_transfer(0x80, 6, 0x0200, 0, 2048)
    except Exception as e:
        print("Error getting descriptor:", e)
        sys.exit(1)
        
    idx = 0
    while idx < len(cfg_desc):
        length = cfg_desc[idx]
        if length == 0:
            break
        desc_type = cfg_desc[idx+1]
        
        # CS_INTERFACE is 0x24
        if desc_type == 0x24:
            desc_subtype = cfg_desc[idx+2]
            # VC_EXTENSION_UNIT is 0x06
            if desc_subtype == 0x06:
                unit_id = cfg_desc[idx+3]
                guid_bytes = cfg_desc[idx+4:idx+20]
                guid_str = ""
                # Format as typical GUID: {Data1-Data2-Data3-Data4}
                # Data1 (4 bytes, little endian)
                d1 = int.from_bytes(guid_bytes[0:4], byteorder='little')
                # Data2 (2 bytes, little endian)
                d2 = int.from_bytes(guid_bytes[4:6], byteorder='little')
                # Data3 (2 bytes, little endian)
                d3 = int.from_bytes(guid_bytes[6:8], byteorder='little')
                # Data4 (8 bytes, as array)
                d4_1 = guid_bytes[8:10]
                d4_2 = guid_bytes[10:16]
                print(f"Found Extension Unit! ID: {unit_id}")
                print(f"GUID: {{{d1:08X}-{d2:04X}-{d3:04X}-{d4_1[0]:02X}{d4_1[1]:02X}-{d4_2[0]:02X}{d4_2[1]:02X}{d4_2[2]:02X}{d4_2[3]:02X}{d4_2[4]:02X}{d4_2[5]:02X}}}")
                
                num_controls = cfg_desc[idx+20]
                print(f"Num Controls: {num_controls}")
        
        idx += length

if __name__ == '__main__':
    parse_descriptors()
