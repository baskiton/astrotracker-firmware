from os.path import isfile

Import('env')
assert isfile('.env')
try:
    envs = []
    with open('.env') as f:
        for line in f:
            envs.append('-D{}'.format(line.strip()))
    env.Append(BUILD_FLAGS=envs)
except IOError:
    print('File .env not accessible')
