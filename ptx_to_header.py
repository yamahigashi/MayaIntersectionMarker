"""Convert a PTX file to a C++ header file with a raw string literal."""
import sys
import os

if len(sys.argv) != 3:
    print(f"Usage: python {sys.argv[0]} <input_ptx_file> <output_header_file>")
    sys.exit(1)

ptx_filename = sys.argv[1]
header_filename = sys.argv[2]
variable_name = os.path.splitext(os.path.basename(ptx_filename))[0] + "_ptx"

try:
    with open(ptx_filename, 'rb') as f_in, open(header_filename, 'w') as f_out:
        f_out.write(f'// Generated from {os.path.basename(ptx_filename)}\n')
        f_out.write('#pragma once\n\n')
        f_out.write(f'const char {variable_name}[] = R"PTX(\n') # Raw string literal start
        content = f_in.read()
        # Decode assuming UTF-8, replace invalid bytes if necessary
        f_out.write(content.decode('utf-8', errors='replace'))
        f_out.write('\n)PTX";\n') # Raw string literal end
    print(f"Successfully converted {ptx_filename} to {header_filename}")
except Exception as e:
    print(f"Error converting {ptx_filename}: {e}")
    sys.exit(1)
