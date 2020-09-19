# step 1

# '''
f = open('./nodelet_temp.csv', 'r')
data = f.readlines()
f.close()

print(len(data))

Prefiltering = []
ScanMatchingOdometry = []
HdlGraphSlam = []
nodelet = []
all = []

i = 0
while i < len(data)-3:
    if data[i].strip()[-1] == 'g':
        j = data[i].strip().split(',')
        j[-1] = float(j[-1][0:-1]) * 1024 * 1024
        data[i] = j[0]+','+j[1]+','+str(j[2])+'\n'
    nodelet.append(data[i])
    Prefiltering.append(data[i+1])
    ScanMatchingOdometry.append(data[i+2])
    HdlGraphSlam.append(data[i+3])
    i += 4

f = open('./Prefiltering.txt', 'a')
for i in Prefiltering:
    f.write(i)
f.close()

f = open('./ScanMatchingOdometry.txt', 'a')
for i in ScanMatchingOdometry:
    f.write(i)
f.close()

f = open('./HdlGraphSlam.txt', 'a')
for i in HdlGraphSlam:
    f.write(i)
f.close()

f = open('./nodelet.txt', 'a')
for i in nodelet:
    f.write(i)
f.close()
# '''

# step 2

'''
f = open('./Prefiltering.txt', 'r')
Prefiltering = f.readlines()
f.close()

f = open('./ScanMatchingOdometry.txt', 'r')
ScanMatchingOdometry = f.readlines()
f.close()

f = open('./HdlGraphSlam.txt', 'r')
HdlGraphSlam = f.readlines()
f.close()

f = open('./nodelet.txt', 'r')
nodelet = f.readlines()
f.close()

all = []
for i in  range(415):
    a = float(Prefiltering[i].strip().split(',')[1])
    b = float(ScanMatchingOdometry[i].strip().split(',')[1])
    c = float(HdlGraphSlam[i].strip().split(',')[1])
    d = float(nodelet[i].strip().split(',')[1])

    e = float(Prefiltering[i].strip().split(',')[2])
    f = float(ScanMatchingOdometry[i].strip().split(',')[2])
    g = float(HdlGraphSlam[i].strip().split(',')[2])
    h = float(nodelet[i].strip().split(',')[2])

    all.append(str(a+b+c+d)+','+str(e+f+g+h)+'\n')

f = open('./all.txt', 'a')
for i in all:
    f.write(i)
f.close()
'''
    

