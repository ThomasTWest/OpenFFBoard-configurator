from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QDialog
from PyQt5.QtWidgets import QWidget
from PyQt5.QtWidgets import QMessageBox,QVBoxLayout,QCheckBox,QButtonGroup 
from PyQt5 import uic
from helper import res_path,classlistToIds
from PyQt5.QtCore import QTimer
import main
from base_ui import WidgetUI

class TMC4671Ui(WidgetUI):

    amp_gain = 60
    shunt_ohm = 0.0015
    
    def __init__(self, main=None):
        WidgetUI.__init__(self, main,'tmc4671_ui.ui')
        #QWidget.__init__(self, parent)
        self.main = main #type: main.MainUi
        self.timer = QTimer(self)
    
        self.pushButton_align.clicked.connect(self.alignEnc)
        self.initUi()
        

        # self.spinBox_tp.valueChanged.connect(lambda v : self.main.serialWrite("torqueP="+str(v)+";"))
        # self.spinBox_ti.valueChanged.connect(lambda v : self.main.serialWrite("torqueI="+str(v)+";"))
        # self.spinBox_fp.valueChanged.connect(lambda v : self.main.serialWrite("fluxP="+str(v)+";"))
        # self.spinBox_fi.valueChanged.connect(lambda v : self.main.serialWrite("fluxI="+str(v)+";"))
        self.spinBox_fluxoffset.valueChanged.connect(lambda v : self.main.serialWrite("fluxoffset="+str(v)+";"))

        self.pushButton_submitmotor.clicked.connect(self.submitMotor)
        self.pushButton_submitpid.clicked.connect(self.submitPid)
        self.main.setSaveBtn(True)
        
        self.timer.timeout.connect(self.updateTimer)

    def __del__(self):
        pass

    def showEvent(self,event):
        self.timer.start(250)

    # Tab is hidden
    def hideEvent(self,event):
        self.timer.stop()
        
    def updateTimer(self):
        if self.main.serialBusy:
            return
        try:
            current = float(self.main.serialGet("acttorque;"))
            v = (2.5/0x7fff) * current
            amps = round((v / self.amp_gain) / self.shunt_ohm,3)
            self.label_Current.setText(str(amps)+"A")

        except Exception as e:
            self.main.log("TMC update error: " + str(e)) 
    

    def submitMotor(self):
        cmd = ""
        mtype = self.comboBox_mtype.currentIndex()
        cmd+="mtype="+str(mtype)+";"

        poles = self.spinBox_poles.value()
        cmd+="poles="+str(poles)+";"

        cmd+="pprtmc="+str(self.spinBox_ppr.value())+";"

        enc = self.comboBox_enc.currentIndex()
        cmd+="encsrc="+str(enc)+";"
        


        self.main.serialWrite(cmd)

    def submitPid(self):
        # PIDs
        cmd = ""
        tp = self.spinBox_tp.value()
        cmd+="torqueP="+str(tp)+";"

        ti = self.spinBox_ti.value()
        cmd+="torqueI="+str(ti)+";"

        fp = self.spinBox_fp.value()
        cmd+="fluxP="+str(fp)+";"

        fi = self.spinBox_fi.value()
        cmd+="fluxI="+str(fi)+";"
        self.main.serialWrite(cmd)


    def initUi(self):
        try:
            # Fill encoder source types
            self.comboBox_enc.clear()
            encsrcs = self.main.serialGet("encsrc!\n")
            for s in encsrcs.split(","):
                e = s.split("=")
                self.comboBox_enc.addItem(e[0],e[1])

            self.getMotor()
            self.getPids()

            self.spinBox_fluxoffset.valueChanged.connect(lambda v : self.main.serialWrite("fluxoffset="+str(v)+";"))
            self.pushButton_submitmotor.clicked.connect(self.submitMotor)
            self.pushButton_submitpid.clicked.connect(self.submitPid)
        except Exception as e:
            self.main.log("Error initializing TMC tab. Please reconnect: " + str(e))
            return False
        return True

    def alignEnc(self):
        res = self.main.serialGet("encalign\n",3000)
        if(res):
            msg = QMessageBox(QMessageBox.Information,"Encoder align",res)
            msg.exec_()

    def getMotor(self):
        res = self.main.serialGet("mtype?;poles?;encsrc?;")
        mtype,poles,enc = [int(s) for s in res.split("\n")]
        if(mtype):
            self.comboBox_mtype.setCurrentIndex((mtype))
        if(poles):
            self.spinBox_poles.setValue((poles))
        if(enc):
            self.comboBox_enc.setCurrentIndex(enc)
            
            self.spinBox_ppr.setValue(int(self.main.serialGet("pprtmc?\n")))
                

    def getPids(self):
        pids = [int(s) for s in self.main.serialGet("torqueP?;torqueI?;fluxP?;fluxI?;fluxoffset?;").split("\n")]
        self.spinBox_tp.setValue(pids[0])
        self.spinBox_ti.setValue(pids[1])
        self.spinBox_fp.setValue(pids[2])
        self.spinBox_fi.setValue(pids[3])

        self.spinBox_fluxoffset.setValue(pids[4])
        