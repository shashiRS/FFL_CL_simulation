import pyrecreader as rec

s = input('press any key')
recPath = "D:\\Projects\\00_General\\SourceCode\\ETK\\04_Engineering\\PYRR_PyRecReader\\04_Engineering\\40_Out\\2.7\\rawrec.rrec"
outRecPath = "D:\\Projects\\00_General\\SourceCode\\ETK\\04_Engineering\\PYRR_PyRecReader\\04_Engineering\\40_Out\\2.7\\rawrec_cut.rrec"

#recPath = u"D:\\Projects\\00_General\\SourceCode\\ETK\\04_Engineering\\PYRR_PyRecReader\\04_Engineering\\40_Out\\2.7\\oldrec.rec"
#outRecPath = u"D:\\Projects\\00_General\\SourceCode\\ETK\\04_Engineering\\PYRR_PyRecReader\\04_Engineering\\40_Out\\2.7\\newrec.rec"

f = rec.open( recPath )
#f = rec.open( u"D:\\Projects\\00_General\\SourceCode\\ETK\\04_Engineering\\PYRR_PyRecReader\\04_Engineering\\90_Deploy\\2.7\\oldrec.rec" )

begints = rec.get_start_timestamp(f)
endts   = rec.get_stop_timestamp(f)

#isOpen  = rec.is_open(f)

deviceCount = rec.get_device_count(f)

#fileName = rec.get_rec_property(f, u'FileName')
#fileGUID = rec.get_rec_property(f, u'GUID')
#extract_session_ok = rec.extract_section(f, outRecPath, 1422449554173495, 1422449575173495)
extract_session_ok = rec.extract_section(f, outRecPath, 784604069, 791604069)


print("Begin TS: {0}".format(begints))
print("End TS: {0}".format(endts))
print("Device Count: {0}".format(deviceCount))
print("Extract Section: {0}".format(extract_session_ok))
