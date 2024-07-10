#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This module trasfer the erg file or mf4 file to Bsig file and mapping the related signals from an Excel Table
"""

import os, xlrd, sys
import numpy as np
from asammdf import MDF
from optparse import OptionParser

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "io"))
from bsig import BsigWriter
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pycmEval"))
from cm_eval_erg_parser import cErgFile

# opens erg file:
_ERG_MDF4_PATH_DEFAULT = os.path.join(os.path.dirname(__file__), "erg_mdf4_2bsig_test")
_MAP_FILE_DEFAULT = os.path.join(_ERG_MDF4_PATH_DEFAULT,  "Signals_Map.xlsx")
_OUTPATH_BSIG_DEFAULT = os.path.join(_ERG_MDF4_PATH_DEFAULT, "test_Bsig_out.bsig")
_MAX_ID_NUMBER = 5000

def take_first(elem):
    return elem[0]

def _find_erg_mdf4_file(path):
    """
    find all erg files under path folder, return the erg files' names in a list
    """
    erg = []
    mf4 = []
    all_files = os.listdir(path)
    if all_files:
        for file in all_files:
            name, ext = os.path.splitext(file)
            if ext == '.erg':
                erg.append(file)
            elif ext == '.mf4':
                mf4.append(file)
    else:
        raise Exception('No files under folder: {}'.format(path))
    return erg, mf4

def _get_map_names(excel_file):
    """
    load mapping list from the excel table
    """
    xl = xlrd.open_workbook(excel_file)
    sheet = xl.sheet_by_index(0)
    signals_list = []
    for row_n in range(1, sheet.nrows):
        signals_list.append(sheet.row_values(row_n))
    return signals_list

def _restructure(sig_list):
    """
    re-structure the signal list from excel table, make it same structure like merge cells
    """
    restru_sig = []
    for sig in sig_list:
        if '\n' in sig[0]:
            sig_split = sig[0].split('\n')
            restru_sig.append([sig_split[0], sig[1], sig[2], sig[3]])
            for idx in range(1, len(sig_split)):
                restru_sig.append([sig_split[idx], '', sig[2], sig[3]])
        else:
            restru_sig.append(sig)
    return restru_sig

def _coeff_offset(value, coefficiant, offset):
    """
    check if there is coefficiant or offset has been given, if yes, multi the coefficiant and plus the offset. Return the values in np.arrary type
    """
    if coefficiant is not '':
        value = value * coefficiant
    if offset is not '':
        value = value + offset
    return value

def _sample_time_filter(target, frq_act, frq_exp):
    """
    filter the values in the target list, depending on the expected sample frequency.
    But if the expected sample frequency bigger than actual sample frequency in the target, the actual frequency will be kept.
    """
    if frq_act > frq_exp:
        n = int(frq_act / frq_exp)
    else:
        n = 1

    if target.ndim == 1:
        filtered = target[::n]
    elif target.ndim == 2:
        tmp = target.T[:, ::n]
        filtered = tmp.T
    else:
        raise Exception("It is not support the order of the list higher than 3.")
    return filtered

def _id_replace(sig_list, name_list, keyword='#ID'):
    """
    replace ID as keyword in signals name as 0, 1, 2 ... until the signal name does not find in the name list. 
    And return the new list with 0, 1, 2, ... signal name.
    """
    new_sig_list = []
    for sig in sig_list:
        if keyword in sig[0]:
            num = sig[0].count(keyword)
            if num == 1:
                for n in range(_MAX_ID_NUMBER):
                    new_name = sig[0].replace(keyword, str(n))
                    if new_name in name_list:
                        if sig[1].count(keyword) == 1:
                            new_name_map = sig[1].replace(keyword, str(n))
                        else:
                            new_name_map = sig[1]
                        new_sig_list.append([new_name, new_name_map, sig[2], sig[3]])
                    else:
                        break
            elif num == 2:
                flag = False
                for j in range(_MAX_ID_NUMBER):
                    for k in range (_MAX_ID_NUMBER):
                        new_name = sig[0][::-1].replace(keyword[::-1], str(k), 1)[::-1].replace(keyword, str(j), 1)
                        if new_name in name_list:
                            if sig[1].count(keyword) == 2:
                                new_name_map = sig[1][::-1].replace(keyword[::-1], str(k), 1)[::-1].replace(keyword, str(j), 1)
                            else:
                                new_name_map = sig[1]
                            new_sig_list.append([new_name, new_name_map, sig[2], sig[3]])
                        elif k != 0:
                            break
                        else:
                            flag = True
                            break
                    if flag:
                        break
            elif num == 3:
                flag_1 = False
                flag_2 = False
                for o in range(_MAX_ID_NUMBER):
                    for p in range (_MAX_ID_NUMBER):
                        for q in range(_MAX_ID_NUMBER):
                            new_name_tmp = sig[0][::-1].replace(keyword[::-1], str(q), 1)[::-1].replace(keyword, str(o), 1)
                            new_name = new_name_tmp.replace(keyword, str(p), 1)
                            if new_name in name_list:
                                if sig[1].count == 3:
                                    new_name_map_tmp = sig[1][::-1].replace(keyword[::-1], str(q), 1)[::-1].replace(keyword, str(o), 1)
                                    new_name_map = new_name_map_tmp.replace(keyword, str(p), 1)
                                else:
                                    new_name_map = sig[1]
                                new_sig_list.append([new_name, new_name_map, sig[2], sig[3]])
                            elif q != 0:
                                break
                            elif p != 0:
                                flag_2 = True
                                break
                            else:
                                flag_1 = True
                                break
                        if flag_1 or flag_2:
                            break
                    if flag_1:
                        break
            else:
                 raise Exception('Not support more than 3 IDs replacement at moment!')
        else:
            new_sig_list.append(sig)
    return new_sig_list

def _erg_mapping(sig_list, erg_file):
    """
    map the signal name from erg to bsig which means the sig_list[][0] to sig_list[][1]. 
    Return the mapped signal name with its values in a dictionary. Meanwhile return the list shows the update signal mapping list with updated ID number for both side.
    """
    fileHandle = cErgFile(erg_file)
    sample_t = round(fileHandle.readAllSignalValues('Time')[1], 6)
    name_list_erg = list(fileHandle._wholeContent.keys())
    sig_new = _id_replace(sig_list, name_list_erg)
    mapped_signals = {}
    fault_signals = []
    bsig_name_list = []
    for sig in sig_new:
        name_erg = sig[0]
        name_bsig = sig[1]
        if name_erg in name_list_erg:
            if name_bsig:
                value = np.array(fileHandle.readAllSignalValues(name_erg))
                value = _coeff_offset(value, sig[2], sig[3])             
            else:
                value_tmp = np.array(fileHandle.readAllSignalValues(name_erg))
                value_tmp = _coeff_offset(value_tmp, sig[2], sig[3])
                value = np.column_stack((value, value_tmp))
                name_bsig = bsig_name_list[-1]
            mapped_signals.update({name_bsig: value})
            bsig_name_list.append(name_bsig)
        else:
            fault_signals.append(sig)
    if fault_signals:
        #print("Those signals {} can't be found in erg file and they have been removed from generated bsig file".format(fault_signals))
        for fault_sig in fault_signals:
            sig_new.remove(fault_sig)
    sig_new.sort(key=take_first)
    return mapped_signals, bsig_name_list, sample_t

def _mdf4_mapping(sig_list, mdf4_file):
    """
    map the signal name from mdf4 to bsig which means the sig_list[][0] to sig_list[][1]. 
    Return the mapped signal name with its values in a dictionary. Meanwhile return the list shows the update signal mapping list with updated ID number for both side.
    """
    mdf4 = MDF(mdf4_file)
    sample_t = round(mdf4.get('Time').samples[1], 6)
    name_list_mdf4 = list(mdf4.channels_db.keys())
    sig_new = _id_replace(sig_list, name_list_mdf4)
    sig_new.sort(key=take_first)
    mapped_signals = {}
    fault_signals = []
    bsig_name_list = []
    for sig in sig_new:
        name_mdf4 = sig[0]
        name_bsig = sig[1]
        if name_mdf4 in name_list_mdf4:
            if name_bsig:
                value = mdf4.get(name_mdf4).samples
                value = _coeff_offset(value, sig[2], sig[3])             
            else:
                value_tmp = mdf4.get(name_mdf4).samples
                value_tmp = _coeff_offset(value_tmp, sig[2], sig[3])
                value = np.column_stack((value, value_tmp))
                name_bsig = bsig_name_list[-1]
            mapped_signals.update({name_bsig: value})
            bsig_name_list.append(name_bsig)
        else:
            fault_signals.append(sig)
    if fault_signals:
        for fault_sig in fault_signals:
            sig_new.remove(fault_sig)
    return mapped_signals, bsig_name_list, sample_t

def _merger_erg_mdf4_2bsig(ergs, mdf4s, path_erg_mf4, signals_map, bsig_file, frq_exp):
    """
    merged all the erg or mf4 files in one fold into on bsig file
    """
    no_ergs = False
    no_mdf4s = False
    with BsigWriter(bsig_file, v2_format=True) as bsig:
        if ergs:
            for erg_file in ergs:
                _mapped, _names, _st_act = _erg_mapping(signals_map, os.path.join(os.path.normpath(path_erg_mf4), erg_file))
                for sig in _names:
                    bsig[sig] = _sample_time_filter(_mapped[sig], int(1/_st_act), frq_exp)
        else:
            no_ergs = True
        if mdf4s:
            for mdf4_file in mdf4s:
                _mapped, _names, _st_act = _mdf4_mapping(signals_map, os.path.join(os.path.normpath(path_erg_mf4), mdf4_file))
                for sig in _names:
                    bsig[sig] = _sample_time_filter(_mapped[sig], int(1/_st_act), frq_exp)
        else:
            no_mdf4s = True
        if no_ergs and no_mdf4s:
            raise Exception("No erg or mdf4 is found in the folder {}".format(path_erg_mf4))
    bsig.close()

def _single_erg_mdf4_2bsig(path_erg_mf4, file_erg_mf4, signals_map, bsig_file, frq_exp):
    """
    take the single erg/mf4 file to convert it to bsig file
    """
    no_erg = False
    no_mdf4 = False
    name, ext = os.path.splitext(file_erg_mf4)
    with BsigWriter(bsig_file, True) as bsig:
        if ext == '.erg':
            _mapped, _names, _st_act = _erg_mapping(signals_map, os.path.join(os.path.normpath(path_erg_mf4), file_erg_mf4))
            for sig in _names:
                bsig[sig] = _sample_time_filter(_mapped[sig], int(1/_st_act), frq_exp)
        else:
            no_erg = True
        if ext == '.mf4':
            _mapped, _names, _st_act = _mdf4_mapping(signals_map, os.path.join(os.path.normpath(path_erg_mf4), file_erg_mf4))
            for sig in _names:
                bsig[sig] = _sample_time_filter(_mapped[sig], int(1/_st_act), frq_exp)
        else:
            no_mdf4 = True
        if no_erg and no_mdf4:
            raise Exception("No correct erg/mf4 has been given")
    bsig.close()

def _get_path_parser():
    """
    run in the console commend in order to get erg/mf4 path or files, the mapping file and output path and its bsig name
    """
    optparser = OptionParser(usage="usage: %prog [options] arg1 arg2 arg3")
    optparser.add_option("-e", "--ergmf4PathFile", dest='erg_mf4', default=_ERG_MDF4_PATH_DEFAULT, help="The path of folder which has erg/mf4 files for merging or specified erg/mf4 file.")
    optparser.add_option("-m", "--mapFile", dest='map_file', default=_MAP_FILE_DEFAULT, help="mapping list file in Excel format with its path")
    optparser.add_option("-o", "--outputFile", dest='out_bsig', default=_OUTPATH_BSIG_DEFAULT, help="the path and name of the output bsig file")
    optparser.add_option("-f", "--samplefrq", dest='sam_frq', default=1000, help="The sample frequency of the bsig signal in Hz")
    erg_mf4_path_file = optparser.parse_args()[0].erg_mf4
    file_mapping = optparser.parse_args()[0].map_file
    bsig_output = optparser.parse_args()[0].out_bsig
    frq = int(optparser.parse_args()[0].sam_frq)
    return erg_mf4_path_file, file_mapping, bsig_output, frq


if __name__ == '__main__':
    erg_mf4, file_mapping, bsig_file, frq_exp = _get_path_parser()
    signals_map = _get_map_names(os.path.normpath(file_mapping))
    signals_map = _restructure(signals_map)
    if os.path.isfile(erg_mf4):
        path_erg_mf4, file_erg_mf4 = os.path.split(erg_mf4)
        _single_erg_mdf4_2bsig(path_erg_mf4, file_erg_mf4, signals_map, bsig_file, frq_exp)
    elif os.path.isdir(erg_mf4):
        ergs, mdf4s = _find_erg_mdf4_file(os.path.normpath(erg_mf4))
        _merger_erg_mdf4_2bsig(ergs, mdf4s, erg_mf4, signals_map, bsig_file, frq_exp)
    else:
        raise Exception("the path of erg/mf4 files or the name of erg/mf4 file has error")





    