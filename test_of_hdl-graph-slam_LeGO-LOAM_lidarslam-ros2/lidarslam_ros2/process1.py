f = open('./temp1.txt', 'r')
data = f.readlines()
f.close()

data_1 = []
temp = ''
for i in range(len(data)):
    if ((i + 1) % 12 ) == 0:
        temp += data[i].strip()
        data_1.append(temp+'\n')
        temp = ''
    else:
        temp += data[i].strip()
        temp += ' '

f = open('./temp3.txt', 'w')
f.writelines(data_1)
f.close()
