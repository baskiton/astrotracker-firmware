import math
import re
import struct
from collections import namedtuple as nt


pfx = '../' if __name__ == '__main__' else ''

cat_dir = pfx + 'data'
HR_CAT_NAME = '/catalogues/hr.cat'
STARNAMES_HR_NAME = '/catalogues/starnames-hr.dat'
MESSIER_CAT_NAME = '/catalogues/messier.cat'
M_NAMES_NAME = '/catalogues/m_names.dat'
NGC_CAT_NAME = '/catalogues/ngc.cat'
IC_CAT_NAME = '/catalogues/ic.cat'
NGCIC_NAMES_NAME = '/catalogues/ngcic_names.dat'
envs = [
    f'-D{HR_CAT_NAME=:s}',
    f'-D{STARNAMES_HR_NAME=:s}',
    f'-D{MESSIER_CAT_NAME=:s}',
    f'-D{M_NAMES_NAME=:s}',
    f'-D{NGC_CAT_NAME=:s}',
    f'-D{IC_CAT_NAME=:s}',
    f'-D{NGCIC_NAMES_NAME=:s}',
]

HrCat = nt('HrCat', 'hr name hd sao v_mag ra dec')
cat_fmt = struct.Struct('<ff')  # ra, dec
# HR_fmt = struct.Struct('<Hff')  # cat_n, ra, dec
MessierCat = nt('MessierCat', 'm name ra dec')
NgcIcCat = nt('NgcIcCat', 'n ra dec')

bayer_num = {
    'Alp': 'α',
    'Bet': 'β',
    'Gam': 'γ',
    'Del': 'δ',
    'Eps': 'ε',
    'Zet': 'ζ',
    'Eta': 'η',
    'The': 'θ',
    'Iot': 'ι',
    'Kap': 'κ',
    'Lam': 'λ',
    'Mu': 'μ',
    'Nu': 'ν',
    'Xi': 'ξ',
    'Omi': 'ο',
    'Pi': 'π',
    'Rho': 'ρ',
    'Sig': 'σ',
    'Tau': 'τ',
    'Ups': 'υ',
    'Phi': 'φ',
    'Chi': 'χ',
    'Psi': 'ψ',
    'Ome': 'ω',
}
exps = {
    '1': '¹',
    '2': '²',
    '3': '³',
    '4': '⁴',
    '5': '⁵',
    '6': '⁶',
    '7': '⁷',
    '8': '⁸',
    '9': '⁹',
}

flam_star_names = []
bay_star_names = []
# filling HR catalogue
with open(cat_dir + HR_CAT_NAME,'w+b') as f:
    n = 1
    for line in open(pfx + 'tools/catalogues/HR-catalog'):
        try:
            ra = int(line[75:77]) * 3600 + int(line[77:79]) * 60 + float(line[79:83])
            dec = int(line[83] + '1') * int(line[84:86]) * 3600 + int(line[86:88]) * 60 + float(line[88:90])
        except:
            # empty coords - exclude from catalogue
            ra = dec = math.nan

        try:
            hd = int(line[25:31])
        except:
            hd = -1

        try:
            sao = int(line[31:37])
        except:
            sao = None

        i = HrCat(
            hr=int(line[0:4]),
            name=line[4:14],
            hd=hd,
            sao=sao,
            v_mag=float(line[102:107].strip() or math.nan),
            ra=ra,     # seconds
            dec=dec     # seconds
        )
        assert (i.hr == n)
        n += 1
        f.write(cat_fmt.pack(i.ra, i.dec))

        if 'NGC' in i.name or 'NOVA' in i.name or i.name.startswith(' M '):
            continue
        num = i.name[:3].strip()
        alp = bayer_num.get(i.name[3:6].strip())
        exp = exps.get(i.name[6].strip(), '')
        con = i.name[7:].strip()
        if any((num, alp, exp, con)):
            if num:     # Flamsteed designation
                flam_star_names.append((f'{num} {con}', i.hr))
            if alp:     # Bayer designation
                bay_star_names.append((f'{alp}{exp} {con}', i.hr))


# filling star names
_star_names = []
names = set()
repeated_names = set()
for line in open(pfx + 'tools/catalogues/STARNAME-EN'):
    hr, _, name = line.strip().partition(' ')
    if name in names:
        repeated_names.add(name)
    else:
        names.add(name)
    _star_names.append((name, int(hr)))

star_names = {}
for i in sorted(_star_names, key=lambda x: x[0][5:] if x[0][0] == "'" else x[0]):
    name = i[0].strip("'").replace("'", "").replace(', ', '|')
    if i[0] in repeated_names:
        name += f' ({i[1]})'
    star_names[name] = i[1]

for i in sorted(bay_star_names, key=lambda x: x[0].strip()):
    if i[0] not in star_names:
        star_names[i[0]] = i[1]

for i in sorted(flam_star_names, key=lambda x: (int(x[0].partition(' ')[0]), x[0].partition(' ')[2].strip())):
    if i[0] not in star_names:
        star_names[i[0]] = i[1]

with open(cat_dir + STARNAMES_HR_NAME, 'wb') as f:
    for v in star_names.values():
        f.write(struct.pack('<H', v))

envs.append(f'-DSTARNAMES=\'{";".join(star_names.keys())}\'')


# Messier
m_names = {}
radec_re = re.compile(r'(?P<sign>[+-]?)(?P<d>\d\d)[h°] (?P<m>.+)[m′]( (?P<s>.+)[s″])?')
with open(cat_dir + MESSIER_CAT_NAME,'w+b') as f:
    for line in open(pfx + 'tools/catalogues/MESSIER'):
        i = line.strip().split('\t')
        m0 = radec_re.match(i[2]).groupdict()
        m1 = radec_re.match(i[3]).groupdict()
        ra = int(m0['d']) * 3600 + float(m0['m']) * 60 + float(m0['s'] or 0)
        dec = int(m1['sign'] + '1') * int(m1['d']) * 3600 + float(m1['m']) * 60 + float(m1['s'] or 0)
        i = MessierCat(int(i[0][1:]), i[1].replace("'", "").replace(', ', '|').replace(' or ', '|'), ra, dec)
        f.write(cat_fmt.pack(i.ra, i.dec))
        if i.name != '-':
            for na in i.name.split('|'):
                m_names[na] = i.m

m_names = dict(sorted(m_names.items()))
with open(cat_dir + M_NAMES_NAME, 'wb') as f:
    for k, v in m_names.items():
        f.write(struct.pack('<H', v))

envs.append(f'-DMESSIER_NAMES=\'{";".join(m_names.keys())}\'')


# NGC/IC
ngc = {}
ic = {}
ngc_re = re.compile(r'(?P<idx>.{5}).{1}(?P<type>.{3}).{1}(?P<ra_h>.{2}).{1}(?P<ra_min>.{4}).{2}'
                    r'(?P<sign>.{1})(?P<dec_d>.{2}).{1}(?P<dec_m>.{2}).{1}(?P<source>.{1}).{2}'
                    r'(?P<const>.{3})(?P<l_size>.{1})(?P<size>.{5}).{2}(?P<mag>.{4})(?P<n_mag>.{1}).{1}'
                    r'(?P<desc>.*)')
for line in open(pfx + 'tools/catalogues/ngc2000'):
    d = ngc_re.match(line).groupdict()

    ra = int(d['ra_h']) * 3600 + float(d['ra_min']) * 60
    dec = int(d['sign'] + '1') * int(d['dec_d']) * 3600 + float(d['dec_m']) * 60

    if d['idx'][0] == 'I':     # IC
        n = int(d['idx'][1:])
        ic[n] = NgcIcCat(n, ra, dec)
    else:   # NGC
        n = int(d['idx'])
        ngc[n] = NgcIcCat(n, ra, dec)

with open(cat_dir + NGC_CAT_NAME,'w+b') as f:
    for k, v in sorted(ngc.items()):
        f.write(cat_fmt.pack(v.ra, v.dec))
    envs.append(f'-DNGC_MAX={k}')
    print('NGC max:', k)

with open(cat_dir + IC_CAT_NAME,'w+b') as f:
    for k, v in sorted(ic.items()):
        f.write(cat_fmt.pack(v.ra, v.dec))
    envs.append(f'-DIC_MAX={k}')
    print('IC max:', k)

# NGCIC_NAMES_NAME
NGCIC_NAMES = []
ngcic_names_re = re.compile(r'(?P<name>.{1,35})(.{1}(?P<ngcic>.{5})(.{1}(?P<comment>.*))?)?')
with open(cat_dir + NGCIC_NAMES_NAME, 'wb') as f:
    for line in open(pfx + 'tools/catalogues/ngc_names'):
        d = ngcic_names_re.match(line).groupdict()
        ngcic = int(d['ngcic'][1:])
        if d['ngcic'][0] == 'I':
            ngcic *= -1
        if d['name'] in NGCIC_NAMES:
            print(d['name'])
        NGCIC_NAMES.append(d['name'].strip().replace("'", ''))
        f.write(struct.pack('<h', ngcic))

    NGCIC_NAMES = ';'.join(NGCIC_NAMES)
    envs.append(f'-D{NGCIC_NAMES=!r}')

# fill .env
with open(pfx + '.env') as f:
    for line in f:
        envs.append('-D{}'.format(line.strip()))


if __name__ != '__main__':
    Import('env')
    env.Append(BUILD_FLAGS=envs)
