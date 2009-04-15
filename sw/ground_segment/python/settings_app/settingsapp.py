#!/usr/bin/env python
#Boa:App:BoaApp

import wx
import os
import settingsframe
import getopt
import sys

sys.path.append(os.getenv("JOBY_PYTHON") + "/joby_lib")
import settings_xml_parse
DEFAULT_AC_IDS = [ 11 ] # biwhirly

def Usage(scmd):
	lpathitem = scmd.split('/')
	fmt = '''Usage: %s [-h | --help] [-a AC_ID | --ac_id=AC_ID]
where
\t-h | --help print this message
\t-a AC_ID | --ac_id=AC_ID where AC_ID is an aircraft ID to use for settings (multiple IDs may passed)
'''
	print fmt   %  lpathitem[-1]

def GetOptions():
  options = {'ac_id':[]}
  try:
    optlist, left_args = getopt.getopt(sys.argv[1:],'h:a:', ['help', 'ac_id='])
  except getopt.GetoptError:
    # print help information and exit:
    Usage(sys.argv[0])
    sys.exit(2)
  for o, a in optlist:
    if o in ("-h", "--help"):
      Usage(sys.argv[0])
      sys.exit()
    elif o in ("-a", "--ac_id"):
      options['ac_id'].append(int(a))

  return options

class SettingsApp(wx.App):
    def OnInit(self):
        options = GetOptions()
        if len(options['ac_id']) > 0:
          ac_ids = options['ac_id']
        else:
          ac_ids = DEFAULT_AC_IDS
        self.main = settingsframe.create(None, ac_ids)
        self.main.Show()
        self.SetTopWindow(self.main)

        return True

def main():
    application = SettingsApp(0)
    application.MainLoop()

if __name__ == '__main__':
    main()
