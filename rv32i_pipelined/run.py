# This code is used to transform dumped machine code from venus (a website that transforms assembly code to machine code)
# to a proper format which is used in the ROM File

ans =[""]
def inst():
    x =input()
    if x == "":
        return 0
    ans[0] = ans[0] + 'X"'+x[6:8]+'", X"'+x[4:6]+'", X"'+x[2:4]+'", X"'+x[0:2]+'",'+"\n"
    inst()

inst()  
print(ans[0])  