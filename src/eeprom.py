import argparse
import struct
import re
import pprint
import binascii
import itertools


parser = argparse.ArgumentParser(description = "AQ32Plus eeprom tool")

parser.add_argument('input', type=argparse.FileType('r'))
parser.add_argument('--raw', nargs='?', type=argparse.FileType('wb'))

args = parser.parse_args()

def parse_eeprom_dump(fd):
	contents = fd.read()
	m = re.search(r"""
		^
		(?P<payload>([A-Za-z0-9]{16,}\n)+)
		\n
		(?P<checksum>[A-Za-z0-9]{8})
		$""", contents, re.M | re.X)

	if not m:
		raise Exception("Could not find eeprom dump.")

	payload = binascii.unhexlify(m.group('payload').replace('\n', ''))
	checksum = m.group('checksum')

	return (payload, checksum)

(payload, checksum) = parse_eeprom_dump(args.input)

print checksum

print '%08X' % (binascii.crc32(payload, 0) & 0xFFFFFFFF)

if args.raw:
	args.raw.write(payload)
	args.raw.close()

def decode_var(v):
	m = re.search(r"""
		^\s*
		(?P<name>[a-zA-Z_][a-zA-Z_0-9]*)
		\s*
		(?P<array>[\[\]a-zA-Z_0-9 ]+)?
		\s*$
		""", v, re.X)

	if not m:
		raise Exception("could not parse variable: '%s'", v)

	name = m.group('name')
	array = m.group('array')
	dims = []

	if array:
		dims = re.findall(r"""
			\[
			\s*
			(?P<dim>[^\]]+)
			\s*
			\]
			""", array, re.X)

	return (name, dims)


def flatten(l):
	return list(itertools.chain.from_iterable(l))

# returns tuples of (type, var_name, [array_dims])
# e.g. ('float', 'accelTCBiasSlope', ['3']);
def decode_struct(file_name, struct_name):
	contents = open(file_name, 'r').read()

	m = re.search(r"""
		typedef\s+struct\s+[a-zA-Z_][a-zA-Z_0-9]*\s*{\s*
		(?P<members>[^}]+)
		\}\s*%s\s*;
		""" % struct_name, contents, re.M | re.X)

	if not m:
		raise Exception("struct regex did not match file '%s', struct '%s'" % 
			(file_name, struct_name))

	members = re.findall(r"""
		^\s*
		(?P<type>[a-zA-Z_][a-zA-Z_0-9]*)
		\s+
		(?P<vars>[^;]+)
		\s*;
		""", m.group('members'), re.M | re.X)

	if len(members) == 0:
		raise Exception("found 0 struct members in file '%s' for struct '%s'" % 
			(file_name, struct_name))

	members = [[(t,) + decode_var(v) for v in vs.split(',')] for (t, vs) in members]

	return flatten(members)

def get_numeric_defines(file_name):
	contents = open(file_name, 'r').read()
	defs = re.findall(r"""
		^\s*
		\#define
		\s+
		(?P<name>[a-zA-Z_][a-zA-Z_0-9]*)
		\s+
		(?P<val>\d+)
		\s*$
		""", contents, re.M | re.X)
	return {n: int(v) for (n, v) in defs}

structs = {
	'PIDdata_t': decode_struct('pid.h', 'PIDdata_t'),
	'eepromConfig_t':  decode_struct('aq32Plus.h', 'eepromConfig_t'),
}

defines = get_numeric_defines('aq32Plus.h')

class BinaryFieldType:
    def __init__(self, name, size, struct_format, display_format):
        self.name = name
        self.size = size
        self.struct_format = struct_format
        self.display_format = display_format

builtins = {t.name: t for t in [
    BinaryFieldType("int8_t",    1, "b", '%s'),
    BinaryFieldType("uint8_t",   1, "B", '%d'),
    BinaryFieldType("int16_t",   2, "h", '%d'),
    BinaryFieldType("uint16_t",  2, "H", '%d'),
    BinaryFieldType("int32_t",   4, "i", '%d'),
    BinaryFieldType("uint32_t",  4, "I", '%d'),
    BinaryFieldType("float",     4, "f", '%.2f')
    ]}

def var_size(size, a):
	for dim in a:
		if dim.isdigit():
			size *= int(dim)
		elif dim in defines:
			size *= defines[dim]
		else:
			raise Exception("'%s' not in defines")

	return size

def alignment_padding(size, align):
	rem = size % align
	return 0 if rem == 0 else align - rem

# returns (strings, total_size)
def print_struct(struct, spaces=''):
	# structs are aligned on this boundary, and take up a multiple of it
	max_align = 4

	if struct not in structs:
		raise Exception("Could not find type '%s' in builtins or structs" % struct)

	members = structs[struct]

	index = 0

	l = []
	for (t, n, a) in members:
		t_size = builtins[t].size if t in builtins else max_align
		index += alignment_padding(index, t_size)
		
		if t in builtins:
			struct_l = []
		else:
			(struct_l, t_size) = print_struct(t, spaces + '  ')

		v_size = var_size(t_size, a)
		l.append('%s%s 0x%x %d' % (spaces, n, index, v_size))
		l.extend(struct_l)
		index += v_size
	
	return (l, index + alignment_padding(index, max_align))
#print list(set(t for name, ms in structs.iteritems() for (t, v, a) in ms))

(l, size) = print_struct('eepromConfig_t')
print '\n'.join(l)
print size
#print get_numeric_defines('aq32Plus.h')

def print_eeprom(eeprom, struct, spaces=''):
	# structs are aligned on this boundary, and take up a multiple of it
	max_align = 4

	if struct not in structs:
		raise Exception("Could not find type '%s' in builtins or structs" % struct)

	members = structs[struct]

	index = 0

	l = []
	for (t, n, a) in members:
		t_size = builtins[t].size if t in builtins else max_align
		index += alignment_padding(index, t_size)

		if t in builtins:
			struct_l = []
		else:
			(struct_l, t_size) = print_struct(t, spaces + '  ')

		v_size = var_size(t_size, a)

		print '%s : ' % n
		
		l.append('%s%s 0x%x %d' % (spaces, n, index, v_size))
		l.extend(struct_l)
		index += v_size
	
	return (l, index + alignment_padding(index, max_align))
