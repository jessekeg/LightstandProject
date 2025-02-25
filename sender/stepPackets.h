#ifndef STEP_PACKETS
  #define STEP_PACKETS


//Modes to send so we don't have to check what data is included
enum modes{
  NONE,
  X_MOTOR,
  Y_MOTOR,
  SYNCHRO
};

//enumerating for "type" in order to define whether we're positioning or accelerating
enum type{
  NO_MODE,
  ACCEL,
  POS
};

typedef struct struct_message {
  modes mode; 
  type Type; //whether to position or accelerate
  int X_DATA;  //to be used either for positioning or acceleration
  int Y_DATA;
} struct_message;

#endif