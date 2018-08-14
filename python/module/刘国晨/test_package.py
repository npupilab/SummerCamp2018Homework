import liuguochen.data_analysis as DataAnalysis
import matplotlib.pyplot as plt

filename = 'data.txt'

analysis=DataAnalysis.DataAnalysis(filename)

I=analysis.plotXY()

subseq = analysis.findMaxSubSequence()

if subseq == [1, 2, 8, 9, 10, 11, 13]:
    print('PASS')
else:
    print('FAILURE')









