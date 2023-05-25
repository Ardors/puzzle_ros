from interfaces.srv import IdentifyPiece

import rclpy
from rclpy.node import Node

from tkinter import *
import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
from PIL import Image
from PIL import ImageTk
import cv2
import imutils
from functools import partialmethod

from ament_index_python.packages import get_package_share_directory
package_sd = get_package_share_directory('py_gui')


class PyGui(Node):

    def __init__(self):
        super().__init__('py_gui')
        self.srv = self.create_service(IdentifyPiece, 'py_vision/IdentifyPiece', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        self.get_logger().info('Incoming request')
        
        response.piece.piece_orientation = 1
        return response
    

    raiz=Tk()
    raiz.title("PUZLE2")
    # raiz.iconbitmap(package_sd+'/ico/upm_logo.ico')

    raiz.geometry("799x532")
    raiz.config(bg="white")

    porcentaje=IntVar()
    
    def armar_conejo(self):
        mensaje=Label(self.raiz,
                    text="PUZLE de Conejo",
                    foreground="brown", 
                    bg="white", 
                    font=("Cooper Black",18))
        mensaje.place(x=380, y=300)

    def armar_elefante(self):
        mensaje=Label(self.raiz,text="PUZLE de Elefante",
                    foreground="brown", 
                    bg="white", 
                    font=("Cooper Black",18))
        mensaje.place(x=380, y=300)

    def armar_jirafa(self):
        mensaje=Label(self.raiz,
                    text="PUZLE de Jirafa",
                    foreground="brown",
                    bg="white", 
                    font=("Cooper Black",18))
        mensaje.place(x=380, y=300)

    def armar_leon(self):
        mensaje=Label(self.raiz,
                    text="PUZLE de León", 
                    foreground="brown", 
                    bg="white", 
                    font=("Cooper Black",18))
        mensaje.place(x=380, y=300)

    def armar_tigre(self):
        mensaje=Label(self.raiz,
                    text="PUZLE de Tigre", 
                    foreground="brown", 
                    bg="white", 
                    font=("Cooper Black",18))
        mensaje.place(x=380, y=300)

    def armar_zebra(self):
        mensaje=Label(self.raiz,
                    text="Puzle de Zebra", 
                    foreground="brown", 
                    bg="white", 
                    font=("Cooper Black",18))
        mensaje.place(x=380, y=300)

    def ingresar_n(self):
        fila = self.entrada_n.get()
        e_fila=Label(self.raiz, 
                    text=fila,
                    foreground="brown",
                    bg="white", 
                    font=("Cooper Black",18))
        e_fila.place(x=620, y=300)


    def ingresar_m(self):
        columna = self.entrada_m.get()
        e_por = Label(self.raiz,
                    text="x",
                    foreground="brown",
                    bg="white", 
                    font=("Cooper Black",18))
        e_por.place(x=640, y=300)
        e_columna=Label(self.raiz, 
                        text= columna,
                        foreground="brown",
                        bg="white", 
                        font=("Cooper Black",18))
        e_columna.place(x=660, y=300)
        


        
    #-------imagen fondo-----------
    img_fondo = PhotoImage(file=package_sd+"/images/fondo.png")
    e_fondo = Label(raiz, image=img_fondo)
    e_fondo.place(x=0, y=0)

    titulo_1 = Label(raiz,text="Bienvenidos a PUZZLE2", 
                foreground="green", 
                bg="white", 
                font=("Cooper Black",24))
    titulo_1.place(x=200, y=10)

    #-------boton conejo-----------
    img_conejo = PhotoImage(file=package_sd+"/images/conejo.png")
    et_conejo = Label(image=img_conejo)

    bt_conejo = Button(raiz,
                    image=img_conejo, 
                    command=partialmethod(armar_conejo))
    bt_conejo.pack(pady=20)
    bt_conejo.place(x=40, y=70)

    #-------boton elefante-----------
    img_elefante = PhotoImage(file=package_sd+"/images/elefante.png")
    et_elefante = Label(image=img_elefante)

    bt_elefante = Button(raiz,
                        image = img_elefante, 
                        command = partialmethod(armar_elefante))
    bt_elefante.pack(pady=20)
    bt_elefante.place(x=190, y=70)

    #-------boton jirafa-----------
    img_jirafa = PhotoImage(file=package_sd+"/images/jirafa.png")
    et_jirafa = Label(image = img_jirafa)

    bt_jirafa=Button(raiz,
                    image = img_jirafa, 
                    command = partialmethod(armar_jirafa))
    bt_jirafa.pack(pady=20)
    bt_jirafa.place(x=40, y=220)

    #-------boton leon-----------
    img_leon = PhotoImage(file=package_sd+"/images/leon.png")
    et_leon = Label(image=img_leon)

    bt_leon=Button(raiz,
                image = img_leon, 
                command = partialmethod(armar_leon))
    bt_leon.pack(pady=20)
    bt_leon.place(x=190, y=220)

    #-------boton tigre-----------
    img_tigre = PhotoImage(file=package_sd+"/images/tigre.png")
    et_tigre = Label(image=img_tigre)

    bt_tigre=Button(raiz,
                    image = img_tigre, 
                    command = partialmethod(armar_tigre))
    bt_tigre.pack(pady=20)
    bt_tigre.place(x=40, y=380)

    #-------boton zebra-----------
    img_zebra = PhotoImage(file=package_sd+"/images/zebra.png")
    et_zebra = Label(image=img_zebra)

    bt_zebra=Button(raiz,image=img_zebra, 
                    command=partialmethod(armar_zebra))
    bt_zebra.pack(pady=20)
    bt_zebra.place(x=190, y=380)


    #------dimensiones--------------
    titulo_2=Label(text="DIMENSIONES",
                foreground="brown", 
                bg="white", 
                font=("Cooper Black",14))
    titulo_2.place(x=380, y=140)

    #filas
    etiqueta_n=Label(text="Filas:",
                    foreground="brown", 
                    bg="white", 
                    font=("Cooper Black",14))
    etiqueta_n.place(x=430, y=174)

    entrada_n=Entry(raiz, width=10)
    entrada_n.place(x=500, y=180)

    img_ok=PhotoImage(file=package_sd+"/images/ok.png")
    et_ok=Label(image=img_ok)

    bt_n=Button(raiz,
                image = img_ok,
                command=partialmethod(ingresar_n))
    bt_n.pack(pady=20)
    bt_n.place(x=570, y=178)

    #columnas
    etiqueta_m=Label(text="Columnas:",
                    foreground="brown", 
                    bg="white", 
                    font=("Cooper Black",14))
    etiqueta_m.place(x=384, y=204)

    entrada_m=Entry(raiz,width=10)
    entrada_m.place(x=500, y=210)

    bt_m=Button(raiz,
                image=img_ok,
                command=partialmethod(ingresar_m))
    bt_m.pack(pady=20)
    bt_m.place(x=570, y=208)

    #-----barra progreso-------------

    barra = ttk.Progressbar(orient = HORIZONTAL,
                        length = 300, 
                        mode = 'determinate')
    barra.place(x=380,y=340) 

    barra ['value'] = 25
    


def main(args=None):
    rclpy.init(args=args)

    minimal_service = PyGui()

    minimal_service.raiz.mainloop()

    # rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()