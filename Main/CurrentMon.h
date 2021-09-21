#include <arduino-timer.h>

class CurrentMon{
    private:

        struct readingDat
        {
            int analogPin;
            int * i;
            uint16_t * arr;
            int sampleNum;
        };

        int analogPin;
        double offset;
        double scale;

        double Irms;
        uint16_t* readings;

        int sampleNum;
        int sampleTime;

        Timer<> timer;

        bool localTimer = false;

    public:

        CurrentMon(double analogPin,double offset, double scale, int sampleNum, int sampleTime, Timer<> t){
            this->analogPin = analogPin ;
            this->offset = offset;
            this->scale  = scale ;
            this->sampleTime = sampleTime;
            this->sampleNum = sampleNum;

            this->timer = t;
        }

        CurrentMon(double offset, double scale):CurrentMon(0, offset, scale, 250, 2, *(new Timer<>)){
            localTimer = true;
        }

        ~CurrentMon(){
            delete(readings);
            if(localTimer){
                delete(&timer);
            }
        }

        float getNewIrms(){
            updateIrms();
            return Irms;
        }

        float getLastIrms(){
            return Irms;
        }

        bool static getReading(readingDat *args){
            args->arr[*(args->i)] = analogRead(args->analogPin);
            //Serial.print("Got reading");
            //Serial.println(*(args->i));
            return ++*(args->i) <= (args->sampleNum);
        }

        bool updateIrms(){
            //Serial.println("Get Irms");
            readings = new uint16_t[sampleNum];

            size_t tasksRunning = timer.size();

            int i = 0;
            readingDat rd;
            rd.analogPin = analogPin;
            rd.i = &i;
            rd.arr = readings;
            rd.sampleNum = sampleNum;

            timer.every(sampleTime, &CurrentMon::getReading, (void *)&rd);
            
            while(i <= sampleNum){//TODO offset by Tinit
                delay(1);
                timer.tick();
                //.Serial.println("Blocking at end");
            }

            for (size_t i = 0; i < sampleNum; i++)
            {
                Irms += readings[i];
            }

            Irms /= sampleNum;
            Irms = pow(Irms, .5);
            Irms = (Irms+offset)*scale;

            //Serial.println("Finished Irms Calc");
            delete(readings);
        }



        void setSampleTime(int sampleTime){
            this->sampleTime = sampleTime;
        }

        void setSampleNum(int sampleNum){
            this->sampleNum = sampleNum;
        }
};
