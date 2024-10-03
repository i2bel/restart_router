#include <ESP8266WiFi.h>
#include <ESP8266Ping.h>
#include <PubSubClient.h>
#include <microDS18B20.h>

const char* ssid     = "xxxxxx"; // подставить название своей сети WiFi
const char* password = "yyyyyyyy"; //подставить пароль свой сети WiFi
//We will use static ip
IPAddress ip(192, 168, 1, 3 );// pick your own suitable static IP address
IPAddress gateway(192, 168, 1, 1); // may be different for your network
IPAddress subnet(255, 255, 255, 0); // may be different for your network (but this one is pretty standard)
IPAddress dns(192, 168, 1, 1);

// MQTT Broker settings
const char *mqtt_broker = "192.168.1.99";  // EMQX broker endpoint
const char *mqtt_topic = "ReBooter/state";     // MQTT topic
const char *mqtt_username = "admin";  // MQTT username for authentication
const char *mqtt_password = "12345678";  // MQTT password for authentication
const int mqtt_port = 1883;  // MQTT port (TCP)
//int MQTT_RECONNECT_INTERVAL 180
//int MQTT_SOCKET_TIMEOUT 120
int MQTT_KEEP_ALIVE = 120; // keep alive time




WiFiClient espClient;
PubSubClient mqtt_client(espClient);
  //espClient.setSocketTimeout(MQTT_SOCKET_TIMEOUT);
 //mqtt_client.setKeepAlive(MQTT_KEEP_ALIVE);



String client_id = "MQTT-ReBooter";
float level = 1;    // Battery voltage in volts
char voltageString[6];

//const char* remote_host_ya = "www.ya.ru"; // первый адрес для пинга
const char* remote_host_go = "pool.ntp.org"; // второй адрес для пинга
const IPAddress remote_ip(1, 1, 1, 1); // адрес для пинга
const IPAddress remote_ip_mqtt(192, 168, 1, 99); // адрес для пинга MQTT брокера

unsigned long previousMillis = 0;
const long interval = 90;     //интервал между пингами в сек
const long T_ROUTER = 60;     // время ожидания загрузки роутера в сек
const long T_RELAY = 5;       // задержка от выключения реле до включения в сек

int PIN_RELAY = D1;
int PIN_LED = 2;
int COUNT_PING = 0;         //количество попыток при отсутствия пинга (не задается)
int COUNT_PING_MAX = 2;     //максимальное количество отсутствия пинга до перезагрузки роутера
int COUNT_WIFI = 0;         //счетчик попыток подключения к WiFi
int COUNT_WIFI_MAX = 120; //300;   // Таймаут до перезагрузки роутера при отсутствии WiFi в сек
MicroDS18B20 <D2> sensor1; // Set temperature sensor on D2

void setup()
{ 
  pinMode(PIN_RELAY, OUTPUT);  // Назначаем первый пин выходом
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_RELAY, HIGH);  // Для управления используем высокий уровень, используем режим реле "нормально открытое"
  // Зажигаем светодиод до тех пор, пока не будет достигнуто успешное подключение по Wi-Fi
  digitalWrite(PIN_LED, LOW);
  Serial.begin(115200);
  delay(1000);
  sensor1.requestTemp();
  sensor1.getTemp();
}

void ConnectWIFI()
// Пока вай-фай не подключен
//   Если количество попыток подключений меньше или равно максимальному количеству
//     Увеличиваем количество попыток на единицу
//   Иначе
//     Перезагружаем роутер
{
  Serial.println();
  Serial.println("Подключаюсь к WiFi"); 
  WiFi.config(ip, dns, gateway, subnet); 
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password); 
   while (WiFi.status() != WL_CONNECTED) 
   {
    if (COUNT_WIFI <= COUNT_WIFI_MAX)
     {
      COUNT_WIFI++;  
     }
    else
     {
      Serial.println();
      Relay();  //перезагружаемся так как нет подключения к WiFi
      Serial.println("Подключаюсь к WiFi"); 
     }
    delay(1000);
    Serial.print(".");
    }
  COUNT_WIFI = 0; // сбрасываем счетчик попыток подключения к WiFi  
  Serial.println();
  Serial.print("WiFi подключен, ip : ");  
  Serial.println(WiFi.localIP());
  MQTT();
  digitalWrite(PIN_LED, HIGH); // Тушим светодиод
  COUNT_PING = 0;   // сбрасываем счетчик неуспешных пингов
}



void MQTT()
{
 //Start MQTT
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setKeepAlive(MQTT_KEEP_ALIVE);
  //mqtt_client.setCallback(mqttCallback);
  connectToMQTTBroker();
  Serial.println("++++++++++++++++");
}


void connectToMQTTBroker() {
   
    if (!mqtt_client.connected()) {
        String client_id = "MQTT-ReBooter";
        Serial.printf("Connecting to MQTT Broker as %s.....\n", client_id.c_str());
        if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Connected to MQTT broker");
            //mqtt_client.subscribe(mqtt_topic);
            // Publish message upon successful connection
            //
            mqtt_client.publish(mqtt_topic, "ReBooter started and connected to MQTT!");
        } else {
            Serial.println("Failed to connect to MQTT broker!");
            //Serial.println(mqtt_client.state());
           // Serial.println(" try again in 5 seconds");
           // delay(5000);
           }
        }
    }

void printValueAndUnits(float value, String units)
{
    Serial.print("   ");
    Serial.print(value);
    Serial.print(" ");
    Serial.println(units);
    //Serial.println("");
}

void Relay()
// Если на первом пине высокий уровень (режим реле нормально открытое)
//   Разрываем цепь NO
{
  if (digitalRead(PIN_RELAY) == HIGH)
  {
    Serial.println("Реле Выключено");
    mqtt_client.publish(mqtt_topic, "Реле Выключено");
    delay(100);
    digitalWrite(PIN_RELAY, LOW);  //отключаем питание на розетке
    }
  delay(T_RELAY*1000);
  digitalWrite(PIN_RELAY, HIGH); //включаем питание на розетке
  Serial.println("Реле Включено, ждем загрузки роутера");
  mqtt_client.publish(mqtt_topic, "Реле Включено, ждем загрузки роутера");
  delay(T_ROUTER*1000); // ждем загрузки роутера
  COUNT_PING = 0;   // сбрасываем счетчик неуспешных пингов
  COUNT_WIFI = 0; // сбрасываем счетчик попыток подключения к WiFi
  previousMillis = 0;
}

void loop() 
{ 
  
  if (WiFi.status() != WL_CONNECTED)  // нет подключения к WiFi
  {
    digitalWrite(PIN_LED, LOW);
    ConnectWIFI(); //подключаемся к WiFi
  }
  unsigned long currentMillis = millis();
  const long X = currentMillis - previousMillis;
  // Условие: Интервал между пингами равен заданному И реле нормально открытое И подключен Wi-Fi
  if (((currentMillis - previousMillis) >= interval*1000) && (digitalRead(PIN_RELAY) == HIGH) && (WiFi.status() == WL_CONNECTED))
  {
    previousMillis = currentMillis;  
   
// MQTT
   
   Serial.print("MQTT_Client_State=");
   Serial.println(mqtt_client.state());
   if ((Ping.ping(remote_ip_mqtt)) && (mqtt_client.state() != 0)) {
    mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password);
    
    if (mqtt_client.state() == 0) {
    Serial.println("ReConnected to MQTT broker!");
    mqtt_client.publish(mqtt_topic, "ReConnected to MQTT broker!");
                                  }
    else {
      Serial.println("Failed to ReConnect to MQTT broker! (Ping to MQTT is OK but no connected!)");
      //Serial.println(mqtt_client.state());
     }
    
    }
   
   else {
         if ((Ping.ping(remote_ip_mqtt)) && (mqtt_client.state() == 0)) {
         Serial.println("Already connected to MQTT broker!"); 
         // Serial.println(mqtt_client.state());
         }
         else {
          Serial.println("Failed to ReConnect to MQTT broker!");
         //Serial.println("Failed to ReConnect to MQTT broker!");
         //Serial.println(mqtt_client.state());
         }
     }
  
    // Voltage meter
    // Reading voltage
    float rawLevel = analogRead(A0);
    level = (float)rawLevel / 10 / (4700. / (65300 + 4700)); // You need to adjust these values according to the voltage divider you install
    Serial.print("Voltage: ");
    printValueAndUnits(level/100, " V");
    mqtt_client.publish("ReBooter/voltage", dtostrf(level/100, 2, 2, voltageString));
    // END VOLTAGE METER

//Temperature meter
 // запрос температуры
  sensor1.requestTemp();
  sensor1.getTemp();
  Serial.print("Temperature: ");
  //Serial.print(sensor1.getTemp());
  // проверяем успешность чтения и выводим
  if (sensor1.readTemp()) {Serial.print(sensor1.getTemp());
  Serial.println("°");
  mqtt_client.publish("ReBooter/temperature", dtostrf((sensor1.getTemp()), 2, 2, voltageString));
    }
  else Serial.println("Error reading temperature!");

//End temperature meter


    Serial.println("Попытка пинга: ");
    mqtt_client.publish(mqtt_topic, "Попытка пинга... ");
    //Serial.println(remote_host_ya);
    Serial.println(remote_host_go);
    Serial.println(remote_ip);
  // Если есть пинг на яндекс ИЛИ гугл
  if((Ping.ping(remote_ip)) || (Ping.ping(remote_host_go)))
  // if  (Ping.ping(remote_host_ya))
   { //пинг есть - моргнули светодиодом
    Serial.println("Пинг есть!!");
    mqtt_client.publish(mqtt_topic, "Пинг есть!");
    digitalWrite(PIN_LED, LOW);
    delay(2000);
    digitalWrite(PIN_LED, HIGH);
    COUNT_PING = 0;// сбрасываем счетчик неуспешных пингов
   } else 
   {  // пинга нет
    Serial.print("Пинг отсутствует :( ");
    mqtt_client.publish(mqtt_topic, "Пинг отсутствует! :(");
    COUNT_PING ++;
    Serial.println(COUNT_PING);
   }
  }
  if (COUNT_PING >= COUNT_PING_MAX) // если превышено количество попыток пинга перезагружаем роутер
  {
     Relay();
  }
    }