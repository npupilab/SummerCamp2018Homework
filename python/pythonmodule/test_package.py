from ***.data_analysis import DataAnalysis


filename = 'data.txt'

analysis=DataAnalysis(filename)

I=analysis.plotXY()

subseq = analysis.findMaxSubSequence()

if subseq == [1, 2, 8, 9, 10, 11, 13]:
    print('PASS')
else:
    print('FAILURE')









