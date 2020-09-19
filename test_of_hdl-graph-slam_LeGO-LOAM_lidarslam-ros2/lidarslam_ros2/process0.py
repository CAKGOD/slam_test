f = open('./32_line_traj_server.txt', 'r')
data = f.readlines()
f.close()

data_1 = []
for i in data:
    if i.strip()[0] not in ['h', 's', 'n', 'f', 'p', '-', 'o', 'w']:
        data_1.append(i.strip()[3:] + '\n')
    else:
        continue

f = open('./temp1.txt', 'w')
f.writelines(data_1)
f.close()
