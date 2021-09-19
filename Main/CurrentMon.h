#include <arduino-timer.h>

class CurrentMon{
    private:

        struct readingDat
        {
            int analogPin;
            int i;
            uint16_t * arr;
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
            args->arr[args->i] = analogRead(args->analogPin);
            return false;
        }

        void getReadingIn(int time, int i){
            int t = 0;

            readingDat rd;
            rd.analogPin = analogPin;
            rd.i = i;
            rd.arr = readings;

            while(timer.in(time-t, &CurrentMon::getReading, (void *)&rd) == NULL){
                delay(1);
                t+=1;
                timer.tick();
            }
        }

        bool updateIrms(){
            readings = new uint16_t[sampleNum];

            size_t tasksRunning = timer.size();

            for (size_t i = 0; i < sampleNum; i++)
            {
                getReadingIn(sampleTime*i, i);
            }
            
            while(timer.size() >= tasksRunning && !timer.empty() && timer.ticks() <= sampleTime*sampleNum){//TODO offset by Tinit
                delay(1);
                timer.tick();
            }

            for (size_t i = 0; i < sampleNum; i++)
            {
                Irms += readings[i];
            }

            Irms /= sampleNum;
            Irms = pow(Irms, .5);
            Irms = (Irms-offset)*scale;

            delete(readings);
        }



        void setSampleTime(int sampleTime){
            this->sampleTime = sampleTime;
        }

        void setSampleNum(int sampleNum){
            this->sampleNum = sampleNum;
        }
};