#include "guipanel.h"
#include "ui_guipanel.h"
#include <QSerialPort>      // Comunicacion por el puerto serie
#include <QSerialPortInfo>  // Comunicacion por el puerto serie
#include <QMessageBox>      // Se deben incluir cabeceras a los componentes que se vayan a crear en la clase
// y que no estén incluidos en el interfaz gráfico. En este caso, la ventana de PopUp <QMessageBox>
// que se muestra al recibir un PING de respuesta

#include<stdint.h>      // Cabecera para usar tipos de enteros con tamaño
#include<stdbool.h>     // Cabecera para usar booleanos

extern "C" {
#include "protocol.h"    // Cabecera de funciones de gestión de tramas; se indica que está en C, ya que QTs
// se integra en C++, y el C puede dar problemas si no se indica.
}

GUIPanel::GUIPanel(QWidget *parent) :  // Constructor de la clase
    QWidget(parent),
    ui(new Ui::GUIPanel)               // Indica que guipanel.ui es el interfaz grafico de la clase
  , transactionCount(0)
{
    ui->setupUi(this);                // Conecta la clase con su interfaz gráfico.
    setWindowTitle(tr("Interfaz de Control")); // Título de la ventana

    // Conexion por el puerto serie-USB
    connected=false;                 // Todavía no hemos establecido la conexión USB
    ui->serialPortComboBox->clear(); // Vacía de componentes la comboBox
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
        // La identificación nos permite que SOLO aparezcan los interfaces tipo USB serial de Texas Instrument
        if ((info.vendorIdentifier()==0x1CBE) && (info.productIdentifier()==0x0002))
        {
            ui->serialPortComboBox->addItem(info.portName());
        }
    ui->serialPortComboBox->setFocus();   // Componente del GUI seleccionado de inicio
    // Las funciones CONNECT son la base del funcionamiento de QT; conectan dos componentes
    // o elementos del sistema; uno que GENERA UNA SEÑAL; y otro que EJECUTA UNA FUNCION (SLOT) al recibir dicha señal.
    // En el ejemplo se conecta la señal readyRead(), que envía el componente que controla el puerto USB serie (serial),
    // con la propia clase PanelGUI, para que ejecute su funcion readRequest() en respuesta.
    // De esa forma, en cuanto el puerto serie esté preparado para leer, se lanza una petición de datos por el
    // puerto serie.El envío de readyRead por parte de "serial" es automatico, sin necesidad de instrucciones
    // del programador
    connect(&serial, SIGNAL(readyRead()), this, SLOT(readRequest()));

    ui->pingButton->setEnabled(false);    // Se deshabilita el botón de ping del interfaz gráfico, hasta que
    // se haya establecido conexión

    //Inicializa la ventana pop-up PING
    ventanaPopUp.setIcon(QMessageBox::Information);
    ventanaPopUp.setText(tr("Status: RESPUESTA A PING RECIBIDA"));
    ventanaPopUp.setStandardButtons(QMessageBox::Ok);
    ventanaPopUp.setWindowTitle(tr("Evento"));
    ventanaPopUp.setParent(this,Qt::Popup);
}

GUIPanel::~GUIPanel() // Destructor de la clase
{
    delete ui;   // Borra el interfaz gráfico asociado a la clase
}

void GUIPanel::readRequest()
{
    int posicion,tam;   // Solo uso notacin hungara en los elementos que se van a
    // intercambiar con el micro - para control de tamaño -
    uint8_t *pui8Frame; // Puntero a zona de memoria donde reside la trama recibida
    uint8_t ui8Command; // Para almacenar el comando de la trama entrante
    
    request.append(serial.readAll()); // Añade el contenido del puerto serie USB al array de bytes 'request'
    // así vamos acumulando  en el array la información que va llegando

    // Busca la posición del primer byte de fin de trama (0xFD) en el array
    posicion=request.indexOf((char)STOP_FRAME_CHAR,0);
    //Metodo poco eficiente pero seguro...
    while (posicion>0)
    {
        pui8Frame=(uint8_t*)request.data(); // Puntero de trama al inicio del array de bytes
        tam=posicion-1;    //Caracter de inicio y fin no cuentan en el tamaño
        // Descarta posibles bytes erroneos que no se correspondan con el byte de inicio de trama
        while (((*pui8Frame)!=(uint8_t)START_FRAME_CHAR)&&(tam>0)) // Casting porque Qt va en C++ (en C no hace falta)
        {
            pui8Frame++;  // Descarta el caracter erroneo
            tam--;    // como parte de la trama recibida
        }
        // A. Disponemos de una trama encapsulada entre dos caracteres de inicio y fin, con un tamaño 'tam'
        if (tam > 0)
        {   //Paquete aparentemente correcto, se puede procesar
            pui8Frame++;  //Quito el byte de arranque (START_FRAME_CHAR, 0xFC)
            //Y proceso normalmente el paquete
            // Paso 1: Destuffing y cálculo del CRC. Si todo va bien, obtengo la trama
            // con valores actualizados y sin bytes de checksum
            tam=destuff_and_check_checksum((unsigned char *)pui8Frame,tam);
            // El tamaño se actualiza (he quitado 2bytes de CRC, mas los de "stuffing")
            if (tam>=0)
            {
                //El paquete está bien, luego procedo a tratarlo.
                ui8Command=decode_command_type(pui8Frame,0); // Obtencion del byte de Comando

                switch(ui8Command) // Segun el comando tengo que hacer cosas distintas
                {
                /** A PARTIR AQUI ES DONDE SE DEBEN AÑADIR NUEVAS RESPUESTAS ANTE LOS COMANDOS QUE SE ENVIEN DESDE LA TIVA **/
                case COMANDO_PING:  // Algunos comandos no tiene parametros
                    // Crea una ventana popup con el mensaje indicado
                    //statusLabel->setText(tr("  RESPUESTA A PING RECIBIDA"));
                    pingResponseReceived();
                    break;
                case COMANDO_BUTTONS:
                {
                    PARAM_COMANDO_BUTTONS parametro;
                    extract_packet_command_param(pui8Frame,sizeof(parametro),&parametro);
                    ui->rightButton->setChecked(parametro.button.fRight);
                    ui->leftButton->setChecked(parametro.button.fLeft);
                    if(parametro.ui8Buttons == 0)
                        ui->led->setEnabled(false);
                    else
                        ui->led->setEnabled(true);
                }
                    break;
                case COMANDO_NO_IMPLEMENTADO:
                {
                    // En otros comandos hay que extraer los parametros de la trama y copiarlos
                    // a una estructura para poder procesar su informacion
                    PARAM_COMANDO_NO_IMPLEMENTADO parametro;
                    extract_packet_command_param(pui8Frame,sizeof(parametro),&parametro);
                    // Muestra en una etiqueta (statuslabel) del GUI el mensaje
                    ui->statusLabel->setText(tr("Status: Comando rechazado,"));
                }
                    break;

                    //Falta por implementar la recepcion de mas tipos de comando

                default:
                    ui->statusLabel->setText(tr("Status: Recibido paquete inesperado,"));
                    break;
                }
            }
        }
        else
        {
            // B. La trama no está completa... no lo procesa, y de momento no digo nada
            ui->statusLabel->setText(tr("Status:Fallo trozo paquete recibido"));
        }
        request.remove(0,posicion+1); // Se elimina todo el trozo de información erroneo del array de bytes
        posicion=request.indexOf((char)STOP_FRAME_CHAR,0); // Y se busca el byte de fin de la siguiente trama
    }
}

void GUIPanel::on_serialPortComboBox_currentIndexChanged(const QString &arg1)
{
    activateRunButton();
}

// Establecimiento de la comunicación USB serie a través del interfaz seleccionado en la comboBox, tras pulsar el
// botón RUN del interfaz gráfico. Se establece una comunicacion a 9600bps 8N1 y sin control de flujo en el objeto
// 'serial' que es el que gestiona la comunicación USB serie en el interfaz QT
void GUIPanel::startSlave()
{
    if (serial.portName() != ui->serialPortComboBox->currentText()) {
        serial.close();
        serial.setPortName(ui->serialPortComboBox->currentText());

        if (!serial.open(QIODevice::ReadWrite)) {
            processError(tr("No puedo abrir el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setBaudRate(9600)) {
            processError(tr("No puedo establecer tasa de 9600bps en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setDataBits(QSerialPort::Data8)) {
            processError(tr("No puedo establecer 8bits de datos en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setParity(QSerialPort::NoParity)) {
            processError(tr("NO puedo establecer parida en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setStopBits(QSerialPort::OneStop)) {
            processError(tr("No puedo establecer 1bitStop en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }

        if (!serial.setFlowControl(QSerialPort::NoFlowControl)) {
            processError(tr("No puedo establecer el control de flujo en el puerto %1, error code %2")
                         .arg(serial.portName()).arg(serial.error()));
            return;
        }
    }

    ui->runButton->setEnabled(false);

    // Se indica que se ha realizado la conexión en la etiqueta 'statusLabel'
    ui->statusLabel->setText(tr("Ejecucion, conectado al puerto %1.")
                             .arg(ui->serialPortComboBox->currentText()));

    // Y se habilitan los controles
    ui->pingButton->setEnabled(true);

    // Variable indicadora de conexión a TRUE, para que se permita enviar comandos en respuesta
    // a eventos del interfaz gráfico
    connected=true;
}

// Funcion auxiliar de procesamiento de errores de comunicación (usada por startSlave)
void GUIPanel::processError(const QString &s)
{
    activateRunButton(); // Activa el botón RUN
    // Muestra en la etiqueta de estado la razón del error (notese la forma de pasar argumentos a la cadena de texto)
    ui->statusLabel->setText(tr("Status: Not running, %1.").arg(s));
}

// Funcion de habilitacion del boton de inicio/conexion
void GUIPanel::activateRunButton()
{
    ui->runButton->setEnabled(true);
}

// SLOT asociada a pulsación del botón RUN
void GUIPanel::on_runButton_clicked()
{
    startSlave();
}

// SLOT asociada a pulsación del botón PING
void GUIPanel::on_pingButton_clicked()
{
    pingDevice();
}

// Funciones de usuario asociadas a la ejecucion de comandos. La estructura va a ser muy parecida en casi todos los
// casos. Se va a crear una trama de un tamaño maximo (100), y se le van a introducir los elementos de
// num_secuencia, comando, y parametros.

// Envío de un comando PING

void GUIPanel::pingDevice()
{
    char paquete[MAX_FRAME_SIZE];
    int size;

    if (connected) // Para que no se intenten enviar datos si la conexion USB no esta activa
    {
        // El comando PING no necesita parametros; de ahí el NULL, y el 0 final.
        // No vamos a usar el mecanismo de numeracion de tramas; pasamos un 0 como n de trama
        size=create_frame((unsigned char *)paquete, COMANDO_PING, NULL, 0, MAX_FRAME_SIZE);
        // Si la trama se creó correctamente, se escribe el paquete por el puerto serie USB
        if (size>0) serial.write(paquete,size);
    }
}

void GUIPanel::pingResponseReceived()

{
    // Ventana popUP para el caso de comando PING; no te deja definirla en un "caso"
    ventanaPopUp.setStyleSheet("background-color: lightgrey");
    ventanaPopUp.setModal(true);
    ventanaPopUp.show();
}

void GUIPanel::on_rojo_stateChanged(int arg1)
{
    cambiaLEDs();
}

void GUIPanel::on_verde_stateChanged(int arg1)
{
    cambiaLEDs();
}

void GUIPanel::on_azul_stateChanged(int arg1)
{
    cambiaLEDs();
}

void GUIPanel::cambiaLEDs(void){
    PARAM_COMANDO_LEDS parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    if(connected)
    {
        // Se rellenan los parametros del paquete (en este caso, el estado de los LED)
        parametro.leds.fRed=ui->rojo->isChecked();
        parametro.leds.fGreen=ui->verde->isChecked();
        parametro.leds.fBlue=ui->azul->isChecked();
        // Se crea la trama con n de secuencia 0; comando COMANDO_LEDS; se le pasa la
        // estructura de parametros, indicando su tamaño; el nº final es el tamaño maximo
        // de trama
        size=create_frame((uint8_t *)pui8Frame, COMANDO_LEDS, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}

void GUIPanel::on_pushButton_clicked()
{
    ui->statusLabel->setText(tr(""));
}

void GUIPanel::on_Knob_valueChanged(double value)
{
    PARAM_COMANDO_BRILLO parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    if(connected)
    {
        // Se rellenan los parametros del paquete (en este caso, el brillo)
        parametro.rIntensity=(float)value;
        // Se crea la trama con n de secuencia 0; comando COMANDO_LEDS; se le pasa la
        // estructura de parametros, indicando su tamaño; el nº final es el tamaño maximo
        // de trama
        size=create_frame((uint8_t *)pui8Frame, COMANDO_BRILLO, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}





void GUIPanel::on_colorWheel_colorChanged(const QColor &arg1)
{
    PARAM_COMANDO_COLOR parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
    if(connected)
    {
        // Se rellenan los parametros del paquete (en este caso, el estado de los LED)
        parametro.leds.fRed=arg1.red();
        parametro.leds.fGreen=arg1.green();
        parametro.leds.fBlue=arg1.blue();
        // Se crea la trama con n de secuencia 0; comando COMANDO_LEDS; se le pasa la
        // estructura de parametros, indicando su tamaño; el nº final es el tamaño maximo
        // de trama
        size=create_frame((uint8_t *)pui8Frame, COMANDO_COLOR, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
        // Se se pudo crear correctamente, se envia la trama
        if (size>0) serial.write((char *)pui8Frame,size);
    }
}

void GUIPanel::on_modo_pwm_clicked(bool checked)
{
    if(checked==true){//es decir, cuando se pulsa para activar
    ui->modo_gpio->click();
    PARAM_COMANDO_MODO parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
        if(connected)
    {
            parametro.modo=0;//pwm
            size=create_frame((uint8_t *)pui8Frame, COMANDO_CAMBIO_MODO, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
            // Se se pudo crear correctamente, se envia la trama
            if (size>0) serial.write((char *)pui8Frame,size);
        }
     }
}





void GUIPanel::on_modo_gpio_clicked(bool checked)
{
    if(checked==true){//es decir, cuando se pulsa para activar
    ui->modo_pwm->click();
    PARAM_COMANDO_MODO parametro;
    uint8_t pui8Frame[MAX_FRAME_SIZE];
    int size;
        if(connected)
    {
            parametro.modo=1;//gpio
            size=create_frame((uint8_t *)pui8Frame, COMANDO_CAMBIO_MODO, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
            // Se se pudo crear correctamente, se envia la trama
            if (size>0) serial.write((char *)pui8Frame,size);
        }
     }
}



