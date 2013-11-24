import os, re, pprint, shutil, sys
import pdb

doxygenModuleTemplate = r'''
/** \addtogroup %s */
/*\@{*/
'''

doxygenCloseModule = r'''
/*\@}*/
'''

doxygenBoardModule = doxygenModuleTemplate % 'Blackboard'

def log(string, file=sys.stdout):
    file.write(string)
    file.write('\n')

def wrapFunction(f):
    def __wrapper(*args, **kargs):
        print('--=<%s>=--' % f.__name__)
        val = f(*args, **kargs)
        print('')
        return val
    return __wrapper

def insertDoxygenModules():
    path = r'D:\thesis\code\CollabExp'
    for root, dirs, files in os.walk(path):
        files = [os.path.join(root, x) for x in files if x.endswith('.h')]
        for file in files:
            if 'Board' in file:
                text = open(file).read()
                f = open(file, 'w')
                f.write(doxygenBoardModule)
                f.write(text)
                f.write(doxygenCloseModule)
                print(file)

def removeDoxygenModules():
    #path = r'D:\thesis\code\CollabExp'
    path = r'C:\Users\declan.obeirne\Documents\dec\th\CollabExp'
    len1 = len(doxygenBoardModule)
    len2 = len(doxygenCloseModule)

    for root, dirs, files in os.walk(path):
        files = [os.path.join(root, x) for x in files if x.endswith('.h')]
        for file in files:
            if 'Board' in file:
                text = open(file).read()
                if text.startswith(doxygenBoardModule) and text.endswith(doxygenCloseModule):
                    text = text[len1:-len2]
                    f = open(file, 'w')
                    f.write(text)
                    print(file)


def removeTempFiles():
    def getFilesToRemove():
        filesToRemove = []
        for root, dirs, files in os.walk(collabExpDir):
            filesToRemove.extend([os.path.join(root, x) for x in files if x.endswith('~')])
        return filesToRemove

    collabExpDir = os.path.normpath(os.path.join(os.getcwd(), os.pardir))
    print('Removing temp files in %s' % collabExpDir)

    filesToRemove = getFilesToRemove()
    print('Files to remove:')
    pprint.pprint(filesToRemove)
    if not filesToRemove:
        return
    for file in filesToRemove:
        os.remove(file)
    filesToRemove = getFilesToRemove()
    print('Remaining files to remove:')
    pprint.pprint(filesToRemove)

def renameImages():
    '''
    Rename imgdata_xxx to imgxxx.
    '''
    zeros = '00000'
    p1 = r'D:\thesis\calibration\images'
    
    for root, dirs, files in os.walk(p1):
        imgs = [(x[len('imgdata_'):], x) for x in files if x.startswith('imgdata_') and x.endswith('.dat')]
        for tail, fullName in imgs:
            import pdb
            #pdb.set_trace()
            lenToPrepend = 9 - len(tail)
            newName = 'img' + zeros[:lenToPrepend] + tail
            open(os.path.join(root, newName), 'wb').write(open(os.path.join(root, fullName), 'rb').read())
    print('Done')

def renameImages2():
    '''
    Rename img    7 to img00007.
    '''
    #dir = r'D:\thesis\calibration\images\robot2_20110812'
    dir = r'D:\thesis\calibration\images\robot2_20110827'
    files = os.listdir(dir)
    for file in files:
        name = file[3:-4]
        num = int(name)
        newFile = 'img%05d.dat' % num
        shutil.copyfile(os.path.join(dir, file), os.path.join(dir, newFile))
    print('Done')

if __name__ == '__main__':
    removeTempFiles()
    #insertDoxygenModules()
    #removeDoxygenModules()
    #renameImages()
    #renameImages2()



    

    