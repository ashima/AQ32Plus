import argparse
import struct
import re
import binascii


parser = argparse.ArgumentParser(description = "AQ32Plus eeprom tool")

parser.add_argument('input', type=argparse.FileType('r'),
	help="serial dump including output from the 'd' command of the EEPROM CLI " +
	     "('U' from main CLI); this doesn't have to be JUST the hex dump -- " +
	     "the important thing is to not mess with the hex dump formatting and " +
	     "ensure it is bounded by newline characters")
parser.add_argument('--raw', nargs='?', type=argparse.FileType('wb'),
	help="output the eepromConfig_t struct in binary, to this file")
parser.add_argument('--exclude-crc', action='store_true',
	help="exclude the CRC when writing to file (use with --raw)")
parser.add_argument('-q', action='store_true', 
	help="output no text, just return 0 for success and nonzero otherwise")

args = parser.parse_args()

def parse_eeprom_dump(fd):
	contents = fd.read()
	m = re.search(r"^([A-Za-z0-9]{16,}(\n\r?|\r\n?))+", contents, re.M | re.X)

	if not m:
		raise Exception("Could not find eeprom dump.")

	eeprom = binascii.unhexlify(m.group(0).replace('\n', '').replace('\r', ''))
	payload  = eeprom[:-4]
	(crc32b,) = struct.unpack('<I', eeprom[-4:])

	return (eeprom, payload, crc32b)

(eeprom, payload, crc32b) = parse_eeprom_dump(args.input)
computed_crc32b = (binascii.crc32(payload, 0) & 0xFFFFFFFF)

if args.raw:
	args.raw.write(payload if args.exclude_crc else eeprom)
	args.raw.close()

if crc32b == computed_crc32b:
	if not args.q:
		print 'CRCs match! (%08x)' % crc32b
	exit(0)
else:
	if not args.q:
		print 'CRC mismatch!'
		print '%08x <-- CRC32B from eeprom dump'   % crc32b
		print '%08x <-- CRC32B from recomputation' % computed_crc32b
	exit(1)

