#!/usr/bin/env python3
"""Biblioteca de nueve modulos para KiCad 10. Huellas con verificaciones pendientes documentadas.
Todos los tamanos en mm. Modelos STEP SIMPLIFICADOS con CadQuery.
"""
from pathlib import Path
import math, csv, json, uuid, re
import cadquery as cq

ROOT=Path(__file__).resolve().parent
FP=ROOT/'Robot_Autonomo.pretty'
ST=ROOT/'Modelos_3D'
FP.mkdir(exist_ok=True); ST.mkdir(exist_ok=True)

# Los nombres de pin son los rotulos de los MODULOS, no los del circuito integrado pelado.
MOD=[]
def mk(name,desc,w,h,left,right=None,source='',status='',notes='',ref='U'):
    d=dict(name=name,desc=desc,w=w,h=h,left=left,right=right or [],source=source,status=status,notes=notes,ref=ref)
    MOD.append(d);return d

def pin(n,label,x,y,typ='bidirectional',drill=.95,sz=1.9):return dict(n=str(n),name=label,x=round(x,3),y=round(y,3),t=typ,drill=drill,sz=sz)

def vert(labels,x,top,step=2.54,start=1,types=None,reverse_num=False,drill=.95,sz=1.9):
    nums=list(range(start,start+len(labels)))
    if reverse_num: nums=nums[::-1]
    return [pin(n,s,x,top+i*step, (types or {}).get(s,'bidirectional'),drill,sz) for i,(n,s) in enumerate(zip(nums,labels))]

def horiz(labels,y,left,step=2.54,start=1,types=None,drill=.95,sz=1.9):
    return [pin(start+i,s,left+i*step,y,(types or {}).get(s,'bidirectional'),drill,sz) for i,s in enumerate(labels)]

S1=['GND','VDD','GND','PC1','PC3','PA1','PA3','PA5','PA7','PC5','PB1','GND','PE7','PE9','PE11','PE13','PE15','PB11','PB13','PB15','PD9','PD11','PD13','PD15','GND']
S2=['GND','VDD','NRST','PC0','PC2','PA0','PA2','PA4','PA6','PC4','PB0','PB2','PE8','PE10','PE12','PE14','PB10','PB12','PB14','PD8','PD10','PD12','PD14','NC','GND']
T1=['GND','5V','3V','PH0','PC14','PE6','PE4','PE2','PE0','PB8','BOOT0','PB6','PB4','PD7','PD5','PD3','PD1','PC12','PC10','PA14','PA10','PA8','PC8','PC6','GND']
T2=['GND','5V','3V','PH1','PC15','PC13','PE5','PE3','PE1','PB9','VDD','PB7','PB5','PB3','PD6','PD4','PD2','PD0','PC11','PA15','PA13','PA9','PC9','PC7','GND']
# ST: P1/P2 arriba hacia abajo; columna 1 = impar, columna 2 = par.
stm=[]
for k in range(25):
    y=round(-48.5+3.34+30.48+2.54*k,3)
    stm += [pin(2*k+1,S1[k],-26.67,y,'power_in' if S1[k] in ('VDD','5V','3V') else ('power_in' if S1[k]=='GND' else 'bidirectional')),
            pin(2*k+2,S2[k],-24.13,y,'power_in' if S2[k] in ('VDD','5V','3V') else ('power_in' if S2[k]=='GND' else 'bidirectional')),
            pin(51+2*k,T1[k],24.13,y,'power_in' if T1[k] in ('VDD','5V','3V','GND') else 'bidirectional'),
            pin(52+2*k,T2[k],26.67,y,'power_in' if T2[k] in ('VDD','5V','3V','GND') else 'bidirectional')]
D=mk('STM32F407G_DISC1','Placa STM32F407G-DISC1; conectores P1 y P2, 2x25 por lado; numeracion interna P1=1-50, P2=51-100',66,97,stm,source='https://www.st.com/resource/en/user_manual/dm00039084.pdf',status='BASE OFICIAL ST: posicion de cabeceras y pinout; confirmar revision fisica',notes='NO soldar con la STM32 en la PCB sin cotejar separacion de cabezales y montaje. Conexiones P2 en archivos CSV.')
D['pstyle']='stm'

# NodeMCU 38 pines: orientacion vista superior, USB abajo; izquierda arriba->abajo (pines 1..19), derecha abajo->arriba (20..38).
L=['GND','GPIO23','GPIO22','GPIO1_TX0','GPIO3_RX0','GPIO21','GND','GPIO19','GPIO18','GPIO5','GPIO17','GPIO16','GPIO4','GPIO0','GPIO2','GPIO15','GPIO8_SD1','GPIO7_SD0','GPIO6_CLK']
R=['3V3','EN','GPIO36_VP','GPIO39_VN','GPIO34','GPIO35','GPIO32','GPIO33','GPIO25','GPIO26','GPIO27','GPIO14','GPIO12','GPIO13','GND','GPIO9_SD2','GPIO10_SD3','GPIO11_CMD','VIN5V']
left=vert(L,-11.43,-22.86,start=1)
right=vert(R,11.43,-22.86,start=20,reverse_num=True)
D=mk('NodeMCU_ESP32S_38P','NodeMCU ESP32S Nubbeo 38 pines CP2102; ESP32-WROOM-32',25.5,48,left,right,
     source='https://www.nubbeo.com.ar/productos/placa-nodemcu-esp32s-wifi-bluetooth-esp32-wroom-arduino-nubbeo/',
     status='PENDIENTE cotejar separacion ENTRE FILAS de 22.86 mm y conexion USB',notes='No apto para fabricar sin medir la separacion entre filas, revisar labels y sentido del pin 1; GPIO6-11 son FLASH.',ref='U')
D['pstyle']='esp'

D=mk('TCRT5000_Modulo_AO_DO','Modulo seguidor de linea TCRT5000 con A0 D0; ancho 14 mm, largo 32 mm nominal',14,32,
     horiz(['VCC','GND','DO','AO'],-13,-3.81,types={'VCC':'power_in','GND':'power_in','DO':'output','AO':'output'}),
     source='https://www.mercadolibre.com.ar/sensor-infrarrojo-seguidor-linea-duaitek-tcrt5000-arduino-salida-analogica-digital/p/MLA32489410',
     status='PROVISIONAL: confirmar orden fisico y posicion del conector',notes='Hay modulos TCRT5000 con distintas posiciones y orden de pines; tambien poseen salida a 5 V en algunos modelos.')
D['pstyle']='tcrt'

D=mk('HC_SR04','Sensor ultrasonico HC-SR04 de 4 pines, 45 x 20 mm',45,20,
     horiz(['VCC','TRIG','ECHO','GND'],8.0,-3.81,types={'VCC':'power_in','GND':'power_in','TRIG':'input','ECHO':'output'}),
     source='https://www.mercadolibre.com.ar/sensor-ultrasonido-hc-sr04-para-arduino-medicion-distancia/p/MLA46841066',
     status='ALTA: cuerpo y pines tipo estandar; confirmar PCB y orientacion del conector',notes='ECHO es normalmente una salida de nivel 5 V; adaptar al STM32/ESP32 si corresponde. Agujeros del sensor no generan agujeros en PCB portadora.')
D['pstyle']='hc'

D=mk('MPU6050_GY521','Modulo MPU6050 GY-521 8 pines I2C (NO chip QFN)',15.5,21,
     vert(['VCC','GND','SCL','SDA','XDA','XCL','AD0','INT'],-6.1,-8.89,types={'VCC':'power_in','GND':'power_in','INT':'output','SCL':'input'}),
     source='https://www.mercadolibre.com.ar/acelerometro-giroscopio-6-ejes-mpu6050-gy-521/p/MLA2039721958',
     status='MEDIA: pinout GY-521 estandar; verificar posicion fila',notes='El modulo GY-521 no es el MPU-6050 pelado. Confirmar tamano 15.5 x 21 y distancias al borde.')
D['pstyle']='mpu'

D=mk('MicroSD_SPI_6P','Modulo lector microSD SPI de 6 pines 42 x 24.2 mm, con regulador/adaptador de nivel',42,24.2,
     vert(['CS','SCK','MOSI','MISO','VCC','GND'],-18.1,-6.35,types={'CS':'input','SCK':'input','MOSI':'input','MISO':'output','VCC':'power_in','GND':'power_in'}),
     source='https://www.mercadolibre.com.ar/modulo-lector-micro-sd-card-5v-con-adaptador-3v3-arduino/p/MLA46823748',
     status='PENDIENTE: conectar y medir fila de 6 pads y orden fisico',notes='Antes de alimentar por 3V3: comprobar regulador/level shifter del modulo especifico; no todos toleran 3V3 en VCC de la placa.')
D['pstyle']='sd'

D=mk('DS1302_RTC_5P','Modulo reloj RTC DS1302 5 pines con portapila CR2032, 43 x 23 mm',43,23,
     vert(['VCC','GND','CLK','DAT','RST'],-18.7,-5.08,types={'VCC':'power_in','GND':'power_in','CLK':'input','DAT':'bidirectional','RST':'input'}),
     source='https://www.mercadolibre.com.ar/modulo-ds1302-rtc-real-time-clock/p/MLA2042233465',
     status='MEDIA: placa tipo MH con 5 pines, verificar taladros de montaje y separacion',notes='Pinout CLK DAT RST para placas MH-Real-Time Clock; otros DS1302 usan otro orden fisico.')
D['pstyle']='ds'

D=mk('TB6612FNG_Breakout_16P','Driver TB6612FNG breakout estilo SparkFun 2 filas x 8 de pines',20.32,22.86,
     vert(['PWMA','AIN2','AIN1','STBY','BIN1','BIN2','PWMB','GND'],-8.89,-8.89,start=1,types={'GND':'power_in'}),
     vert(['VM','VCC','GND','AO1','AO2','BO2','BO1','GND'],8.89,-8.89,start=9,reverse_num=True,types={'VM':'power_in','VCC':'power_in','GND':'power_in','AO1':'output','AO2':'output','BO1':'output','BO2':'output'}),
     source='https://www.mercadolibre.com.ar/modulo-driver-motor-paso-a-paso-tb6612fng-parduino-emakers/p/MLA2040552181',
     status='MEDIA: pinout SparkFun confirmado, ancho/alto y separacion columnas por revisar',notes='VCC=logica; VM=motores; STBY debe estar habilitado. Confirmar si modulo comercial coincide con SparkFun.')
D['pstyle']='tb'

D=mk('StepDown_Mini_3A_0V8_17V','Step Down mini 3 A 0.8-17V fijo/ajustable de 20 x 11 mm, EN IN+ GND VO+',20,11,
     vert(['EN','VIN','GND','VOUT'],-8.25,-3.81,types={'EN':'input','VIN':'power_in','GND':'power_in','VOUT':'power_out'},drill=.8,sz=1.45),
     source='https://www.mercadolibre.com.ar/step-down-salida-fija-y-ajustable-08-17v-mini-3a-pack-x-3/p/MLA2084406221',
     status='MEDIA: producto 20 x 11 mm y pines 4x 2.54 segun ficha comercial; medir centros/EN',notes='Este no es MP1584. Maximo 3 A pico no garantiza 3 A sostenidos; medir temperatura y corriente. EN a GND apaga.')
D['pstyle']='buck'


def esc(s):return str(s).replace('\\','\\\\').replace('"','\\"').replace('\n',' ')
def fmt(x):return ('%.4f'%x).rstrip('0').rstrip('.') if isinstance(x,(float,int)) else str(x)

def pin_type(p):
    if p['name'] in ('GND','VCC','VDD','VM','3V3','3V','5V','VIN','VIN5V'): return 'power_in'
    if p['name'] in ('OUT','VOUT','VO+'): return 'power_out'
    if p['name'] in ('NC',): return 'no_connect'
    return p['t']

def sym(d):
    ps=d['left']+d['right']
    if d['name']=='STM32F407G_DISC1':
        # Pinout mas legible en dos columnas P1 y P2, 50 pines cada lado
        a=sorted((p for p in ps if int(p['n'])<=50),key=lambda z:int(z['n']))
        b=sorted((p for p in ps if int(p['n'])>50),key=lambda z:int(z['n']))
    elif d['right']:
        a=sorted(d['left'],key=lambda z:int(z['n']))
        b=sorted(d['right'],key=lambda z:int(z['n']))
    else:
        a=sorted(ps,key=lambda z:int(z['n']))
        b=[]
    nmax=max(len(a),len(b),5)
    pitch=2.54
    halfheight=math.ceil((nmax+1)*pitch/2.54)*1.27
    halfwidth=25.4 if d['name']=='STM32F407G_DISC1' else (20.32 if d['name']=='NodeMCU_ESP32S_38P' else 15.24)
    p_x=halfwidth+5.08
    lines=[]
    for side,arr in [('L',a),('R',b)]:
        for k,p in enumerate(arr):
            y=round(((len(arr)-1)/2-k)*pitch,4)
            # colocar en grilla 1.27mm
            angle=0 if side=='L' else 180
            x=-p_x if side=='L' else p_x
            name=p['name']
            if d['name']=='STM32F407G_DISC1':
                prefix='P1' if int(p['n'])<=50 else 'P2'
                ix=int(p['n']) if prefix=='P1' else int(p['n'])-50
                name=f'{prefix}.{ix:02d}_{p["name"]}'
            lines.append(f'''      (pin {pin_type(p)} line (at {fmt(x)} {fmt(y)} {angle}) (length 5.08)
        (name "{esc(name)}" (effects (font (size 1.016 1.016))))
        (number "{p['n']}" (effects (font (size 1.016 1.016)))))''')
    body=f'''    (symbol "{d['name']}_0_1"
      (rectangle (start {-halfwidth} {fmt(halfheight)}) (end {halfwidth} {fmt(-halfheight)})
        (stroke (width 0.254) (type default)) (fill (type background))))'''
    namepin=f'''    (symbol "{d['name']}_1_1"
{chr(10).join(lines)}
    )'''
    src=d['source']
    return f'''  (symbol "{d['name']}"
    (pin_names (offset 1.016))
    (in_bom yes) (on_board yes)
    (property "Reference" "{d['ref']}" (at 0 {fmt(halfheight+5.08)} 0)
      (effects (font (size 1.27 1.27))))
    (property "Value" "{d['name']}" (at 0 {fmt(halfheight+2.54)} 0)
      (effects (font (size 1.27 1.27))))
    (property "Footprint" "Robot_Autonomo:{d['name']}" (at 0 {-halfheight-2.54} 0)
      (effects (font (size 1.27 1.27)) (hide yes)))
    (property "Datasheet" "{esc(src)}" (at 0 {-halfheight-5.08} 0)
      (effects (font (size 1.27 1.27)) (hide yes)))
    (property "Description" "{esc(d['desc'])}" (at 0 {-halfheight-7.62} 0)
      (effects (font (size 1.27 1.27)) (hide yes)))
{body}
{namepin}
  )'''

syms=['(kicad_symbol_lib','  (version 20231120)','  (generator "robot_autonomo_python")']
for d in MOD:syms.append(sym(d))
syms.append(')')
(ROOT/'Robot_Autonomo.kicad_sym').write_text('\n'.join(syms)+'\n',encoding='utf8')


def fp_text(user_text,x,y,layer,size=1):
    return f'''  (fp_text user "{esc(user_text)}" (at {fmt(x)} {fmt(y)}) (layer "{layer}")
    (effects (font (size {fmt(size)} {fmt(size)}) (thickness 0.12))))'''
def rect(w,h,layer,stroke=.12):
    return f'  (fp_rect (start {fmt(-w/2)} {fmt(-h/2)}) (end {fmt(w/2)} {fmt(h/2)}) (stroke (width {fmt(stroke)}) (type default)) (fill none) (layer "{layer}"))'
def circ(x,y,r,layer='F.Fab'):
    return f'  (fp_circle (center {fmt(x)} {fmt(y)}) (end {fmt(x+r)} {fmt(y)}) (stroke (width 0.12) (type default)) (fill none) (layer "{layer}"))'
def fpfile(d):
    w,h=d['w'],d['h']
    parts=[f'(footprint "{d["name"]}"','  (version 20240108)','  (generator "robot_autonomo_python")','  (layer "F.Cu")',f'  (descr "{esc(d["desc"])}; {esc(d["status"])}")','  (tags "Robot Autonomo Modulo placa adaptador")', '  (attr through_hole)']
    parts.append(f'''  (property "Reference" "REF**" (at 0 {fmt(-h/2-3.6)} 0) (layer "F.SilkS")
    (effects (font (size 1 1) (thickness 0.15))))''')
    parts.append(f'''  (property "Value" "{d['name']}" (at 0 {fmt(h/2+3.5)} 0) (layer "F.Fab")
    (effects (font (size 1 1) (thickness 0.15))))''')
    parts.append(rect(w,h,'F.Fab',.12))
    parts.append(rect(w+1,h+1,'F.CrtYd',.05))
    parts.append(rect(w,h,'F.SilkS',.12))
    # esquina pin 1
    p1=next(p for p in d['left']+d['right'] if p['n']=='1')
    parts.append(circ(p1['x'],p1['y'],1.3,'F.Fab'))
    # elementos 2D orientativos (NO agujerear carrier PCB por agujeros de la placa hija)
    if d['pstyle']=='hc':
        for x in (-13.5,13.5):parts.append(circ(x,-2,8,'F.Fab'))
        for x in (-20,20):
            for y in (-8.2,8.2):parts.append(circ(x,y,1.1,'F.Fab'))
    if d['pstyle']=='ds':parts.append(circ(8,0,10,'F.Fab'))
    if d['pstyle']=='sd':parts.append(rect(16,15,'F.Fab',.1))
    if d['pstyle']=='esp':parts.append(fp_text('USB',0,21,'F.Fab',1))
    if d['pstyle']=='stm':
        parts.append(fp_text('P1',-25.4,-17.5,'F.Fab',1.3))
        parts.append(fp_text('P2',25.4,-17.5,'F.Fab',1.3))
    parts.append(fp_text('1',p1['x']-2.3,p1['y']-1.8,'F.SilkS',.8))
    # pines de header o orificios. Todo en el formato exacto de KiCad.
    for p in d['left']+d['right']:
        x,y=p['x'],p['y'];shape='rect' if p['n']=='1' else 'circle'
        parts.append(f'  (pad "{p["n"]}" thru_hole {shape} (at {fmt(x)} {fmt(y)}) (size {fmt(p["sz"])} {fmt(p["sz"])}) (drill {fmt(p["drill"])}) (layers "*.Cu" "*.Mask"))')
    parts.append(f'''  (model "${{KIPRJMOD}}/Librerias/Modelos_3D/{d['name']}.step"
    (offset (xyz 0 0 0)) (scale (xyz 1 1 1)) (rotate (xyz 0 0 0)))''')
    parts.append(')')
    (FP/(d['name']+'.kicad_mod')).write_text('\n'.join(parts)+'\n',encoding='utf8')

for d in MOD:fpfile(d)
# Las tablas activas viven junto al .kicad_pro y se mantienen por separado.

# Modelo 3D SOLIDO pero SIMPLIFICADO, no es render/calco del modulo real.
PCB_H=1.6
colors={'board':cq.Color(0.07,0.41,0.24),'blue':cq.Color(.07,.15,.70),'red':cq.Color(.76,.09,.08),'black':cq.Color(.09,.105,.12),'metal':cq.Color(.72,.76,.78),'gold':cq.Color(.81,.62,.18),'white':cq.Color(.89,.89,.85),'dark':cq.Color(.18,.20,.22)}

def boxassy(asm,x,y,z,w,h,dep,color,tag):
    solid=cq.Workplane('XY').box(w,h,dep).translate((x,y,z+dep/2)).val()
    asm.add(solid,name=f'{tag}_{len(asm.children)}',color=colors[color])

def cylassy(asm,x,y,z,r,dep,color,tag):
    solid=cq.Workplane('XY').circle(r).extrude(dep).translate((x,y,z)).val()
    asm.add(solid,name=f'{tag}_{len(asm.children)}',color=colors[color])

def make_step(d):
    name=d['name'];sty=d['pstyle']
    asm=cq.Assembly(name=name)
    # Placas enchufadas con pines: cara inferior z~3 mm sobre la portadora.
    z0=3.3
    pcbcolor='red' if sty=='tb' else ('blue' if sty in ('tcrt','sd','mpu') else 'board')
    boxassy(asm,0,0,z0,d['w'],d['h'],PCB_H,pcbcolor,'PCB_geometry_approx')
    # Modelado simplificado de header/pad metal - todos los pin finales en XY exacto de huella.
    for j,p in enumerate(d['left']+d['right']):
        x,y=p['x'],p['y'];
        boxassy(asm,x,y,0,0.62,0.62,7,'gold',f'Header_{p["n"]}')
    if sty=='stm':
        boxassy(asm,0,9,z0+PCB_H,14,14,2,'black','STM32F407_Chip')
        boxassy(asm,0,-27,z0+PCB_H,12,12,2,'black','STLINK_Chip')
        boxassy(asm,0,-46,z0+PCB_H,8,6,3.5,'metal','STLINK_USB')
        boxassy(asm,0,45,z0+PCB_H,8,6,3.5,'metal','USB_OTG')
        cylassy(asm,-15,45,z0+PCB_H,3.1,7,'black','AudioJack')
        for a,(x,y) in enumerate([(10,17),(15,12),(10,7),(5,12)]):cylassy(asm,x,y,z0+PCB_H,1.5,.8,'white',f'LED{a}')
    elif sty=='esp':
        boxassy(asm,0,-9,z0+PCB_H,17.5,24,2.8,'metal','ESP32_RF_Shield')
        boxassy(asm,0,-20.5,z0+PCB_H+2.8,15,5,.2,'gold','ESP32_Antenna')
        boxassy(asm,0,24,z0+PCB_H,8,6,3,'metal','USB_Approx')
        for x in [-8,8]:boxassy(asm,x,13,z0+PCB_H,3,3,1.8,'black','Button')
    elif sty=='tcrt':
        boxassy(asm,0,10,z0+PCB_H,5.8,6.8,5.5,'black','TCRT5000_optical')
        boxassy(asm,0,0,z0+PCB_H,5.5,5.5,3.4,'blue','Potentiometer')
        boxassy(asm,0,-5,z0+PCB_H,5,3,1.5,'black','Comparator')
    elif sty=='hc':
        for x in [-13.5,13.5]:
            cylassy(asm,x,-2,z0+PCB_H,8,11,'metal','US_transducer_can')
            cylassy(asm,x,-2,z0+PCB_H+11,6.2,.3,'dark','US_transducer_face')
        boxassy(asm,0,6,z0+PCB_H,5,3,1.1,'black','Controller')
    elif sty=='mpu':
        boxassy(asm,2,0,z0+PCB_H,4.2,4.2,1.2,'black','MPU6050')
        boxassy(asm,2,-7,z0+PCB_H,3,2,1.4,'black','Regulator')
    elif sty=='sd':
        boxassy(asm,6,0,z0+PCB_H,18,15,1.9,'metal','microSD_CardHolder')
        boxassy(asm,-8,4,z0+PCB_H,5,6,1.4,'black','LevelShifter')
    elif sty=='ds':
        cylassy(asm,8,0,z0+PCB_H,10,2.4,'metal','CR2032_Holder')
        cylassy(asm,8,0,z0+PCB_H+2.4,8.8,.5,'white','CR2032_Cell')
        boxassy(asm,-8,-1,z0+PCB_H,4,9,2,'black','DS1302')
    elif sty=='tb':
        boxassy(asm,0,0,z0+PCB_H,7,7,1.4,'black','TB6612FNG')
        for x in [-4,4]:
            boxassy(asm,x,-6,z0+PCB_H,2,1.5,.5,'white','Decoupling')
    elif sty=='buck':
        boxassy(asm,1,-.5,z0+PCB_H,5,5,3.4,'dark','Inductor')
        boxassy(asm,6,2,z0+PCB_H,3,3,2,'blue','Adjuster')
        boxassy(asm,-3,2,z0+PCB_H,3.5,2.8,.7,'black','Converter_IC')
    filepath=ST/(name+'.step')
    asm.export(str(filepath),exportType='STEP',mode='default')
    return filepath
for i,d in enumerate(MOD,1):
    filepath=make_step(d)
    print(f'[{i}/9] {d["name"]}: {filepath.stat().st_size} bytes STEP')

with (ROOT/'Documentacion'/'Mapeo_pines.csv').open('w',newline='',encoding='utf8') as f:
    wr=csv.writer(f);wr.writerow(['Componente','Numero de pad/simbolo','Nombre','X_mm','Y_mm','Tipo_electrico'])
    for d in MOD:
        for p in sorted(d['left']+d['right'],key=lambda t:int(t['n'])):
            wr.writerow([d['name'],p['n'],p['name'],p['x'],p['y'],pin_type(p)])

with (ROOT/'Documentacion'/'Verificaciones.csv').open('w',newline='',encoding='utf8') as f:
    wr=csv.writer(f);wr.writerow(['Componente','Dimensiones_placa_mm','Estado','Verificaciones_pendientes','Fuente_publica'])
    for d in MOD:wr.writerow([d['name'],f'{d["w"]} x {d["h"]}',d['status'],d['notes'],d['source']])

readme='''# Robot_Autonomo — bibliotecas KiCad 10

**Estado: PRELIMINAR, no liberado para fabricar.** Se incluyen los 9 simbolos, 9 huellas y 9 STEP simplificados. Los STEP son modelos mecanicos *conceptuales*; no son ficheros originales de los fabricantes ni permiten comprobar colisiones de todos los componentes SMD reales. **No enviar el PCB a fabricar sin verificar cada huella contra las placas fisicas.**

## Instalacion como biblioteca del proyecto

1. Abrir `PCB_Robot_Autonomo.kicad_pro` en KiCad 10, manteniendo `Librerias/` junto al archivo de proyecto.
2. Las tablas `sym-lib-table` y `fp-lib-table` de la raíz registran las bibliotecas locales. Desde Preferencias → Administrar bibliotecas de símbolos o huellas se pueden consultar en el alcance **Proyecto**.
3. Editor de esquemas → Agregar simbolo (`A`) → buscar `Robot_Autonomo` y seleccionar el componente.
4. En el simbolo ya queda asignada `Robot_Autonomo:<nombre>`. Actualizar PCB desde esquema (`F8`).
5. Ver → Visor 3D. Las huellas de `Robot_Autonomo.pretty` usan `${KIPRJMOD}/Librerias/Modelos_3D/`. La PCB también referencia modelos originales en `Librerias/Modelos 3d/` y modelos estándar de KiCad 10.

**Sobre el STM32:** Los 100 pads de la huella se numeran `1-50` para `P1.1-P1.50` y `51-100` para `P2.1-P2.50`. Ver `Documentacion/Mapeo_pines.csv`; por ejemplo pad `56` = `P2.6`. No son números de patas físicas del MCU LQFP100, sino de las cabeceras de expansión de la placa de desarrollo.

## Interruptor SW1

Las tablas activas incluyen `Robot_Interruptores`, con el símbolo
`SW_MTS102_A2_SPDT` y la huella `SW_MTS102_A2_PCB_P4.70mm`.
La PCB usa esa huella para SW1, mientras que el esquema conserva un símbolo
SPST de dos pines con una asignación de huella distinta. Revisar la correspondencia
de pines y pads antes de sincronizar la PCB desde el esquema. El filtro por
cantidad de pines puede ocultar una huella de tres pads para un símbolo de dos.

La huella se basó en la MTS-102-A2. La compatibilidad con la STM102-CI comercial
queda por verificar con el plano o la pieza adquirida antes de fabricar.

## Niveles de evidencia y dimensiones

| Huella | Estado preliminar |
| --- | --- |
| STM32F407G_DISC1 | Plano mecanico ST UM1472 para 66x97mm y headers 2x25: posicionado segun la figura 5; revisar revision exacta. |
| NodeMCU_ESP32S_38P | Nubbeo reporta 48x25.5mm / 38 pines. **Separacion de filas supuesta 22.86 mm**; confirmar con calibre. |
| TCRT5000_Modulo_AO_DO | Placa 14x32 y 4 contactos: cuerpo/orden orientativo, comprobar modulo Duaitek. |
| HC_SR04 | Modulo estandar 45x20, 4 pines 2.54, comprobar orientacion y medida real. |
| MPU6050_GY521 | Tipo GY-521 8 pines. Verificar borde, pin1 y taladros. |
| MicroSD_SPI_6P | Lector SPI 42x24.2 tipo azul; pinout y ubicacion son ORIENTATIVOS. |
| DS1302_RTC_5P | Tipo 43x23 5 pines MH; comprobar posiciones de pins y tornillos. |
| TB6612FNG_Breakout_16P | Distribucion de senales de placa tipo SparkFun, **distancia entre columnas y largo preliminares**. |
| StepDown_Mini_3A_0V8_17V | **20x11** de ficha de INFORMATICA SAN ISIDRO, no MP1584; 4 pines EN/VIN/GND/VO+, revisar distancias al borde. |

## Riesgos electricos

- HC-SR04: ECHO puede entregar 5 V; interponer adaptacion de niveles antes de una entrada MCU 3.3 V que no sea tolerante.
- ESP32 NodeMCU: GPIO6-GPIO11 son usados normalmente por la memoria flash; no son pines GPIO de uso general recomendado.
- MicroSD: los modulos con regulador de 5 a 3.3 V y adaptadores de nivel varian, verificar si VCC=3.3 V es admisible en la placa exacta.
- TB6612: VM es alimentacion de motores, VCC de logica, y STBY se debe habilitar para mover los motores.
- Step Down: 3 A es pico y depende de refrigeracion; confirmar maximo continuo y configuracion de voltaje con carga.

## De donde vienen los datos

- STM: ST UM1472 https://www.st.com/resource/en/user_manual/dm00039084.pdf (pag 15 y tabla de conectores).
- ESP32: https://www.nubbeo.com.ar/productos/placa-nodemcu-esp32s-wifi-bluetooth-esp32-wroom-arduino-nubbeo/ y pinout NodeMCU-32S comun.
- TB6612: https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide (rotulos de modulo)
- Step Down mini: https://www.mercadolibre.com.ar/step-down-salida-fija-y-ajustable-08-17v-mini-3a-pack-x-3/p/MLA2050102871 y ficha placa de 20x11.
- Otros: enlaces particulares incluidos en `Documentacion/Verificaciones.csv`.

## Criterios mecanicos

- Posiciones XY dadas en el CSV (vistas desde arriba como se pondria el modulo sobre la placa portadora).
- Los taladros de montaje del MODULO no se perforan automaticamente en la PCB madre, solo se dibujan referencias visuales si corresponde.
- Los pads de 2.54 mm son THT para insertar pines de modulos; verificar taladro y diametro segun tiras de pines reales (tipicamente 0.95mm y 1.90mm de pad). El Step Down usa taladro 0.8 y pad 1.45mm para agujeros finos.
- Los modelos STEP conservan posiciones relativas XY de los pads pero se representan con hardware simplificado. Su altura relativa sobre la PCB portadora es aproximada.

## Archivos

- `Robot_Autonomo.kicad_sym`: 9 simbolos.
- `Robot_Autonomo.pretty/`: 9 huellas (1 archivo `.kicad_mod` cada una).
- `Modelos_3D/`: 9 modelos `.step` SIMPLIFICADOS.
- `../sym-lib-table` y `../fp-lib-table`: tablas activas del proyecto.
- `Documentacion/Mapeo_pines.csv`: numeros y nombres de las conexiones.
- `Documentacion/Verificaciones.csv`: nivel de verificacion y dudas fisicas.
- `generar_bibliotecas.py`: script editable para corregir las posiciones tras medir los modulos.

Este paquete no sustituye la comprobacion del hardware que se comprara/ya se compro. Revisar con regla/calibre antes de fabricar.
'''
(ROOT/'LEEME_Instalacion_y_Verificacion.md').write_text(readme,encoding='utf8')

# Validacion estructural sin kicad-cli (ausente en este entorno)
for p in [ROOT/'Robot_Autonomo.kicad_sym',*FP.glob('*.kicad_mod'),ROOT.parent/'sym-lib-table',ROOT.parent/'fp-lib-table']:
    s=p.read_text();depth=0;quote=False;escape=False
    for c in s:
        if quote:
            if escape:escape=False
            elif c=='\\':escape=True
            elif c=='"':quote=False
        elif c=='"':quote=True
        elif c=='(':depth+=1
        elif c==')':depth-=1
        if depth<0:raise ValueError('Error parentesis '+str(p))
    if depth or quote:raise ValueError('Error sintaxis '+str(p))
for d in MOD:
    numbers=[p['n'] for p in d['left']+d['right']]
    assert len(numbers)==len(set(numbers))
    assert (FP/(d['name']+'.kicad_mod')).exists() and (ST/(d['name']+'.step')).exists()
print(f'OK: {len(MOD)} simbolos, {len(list(FP.glob("*.kicad_mod")))} huellas y {len(list(ST.glob("*.step")))} STEP. Estructura balanceada.')
