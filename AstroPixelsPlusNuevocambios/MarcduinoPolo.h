static void SetupPolo(){
    pinMode(PIN_AUX1,OUTPUT);  //PERISCOPIO
    pinMode(PIN_AUX2,OUTPUT);  //BAD MOTIVADOR
    pinMode(PIN_AUX3,OUTPUT);  //Zapper
    pinMode(PIN_AUX4,OUTPUT);  //Sable Luz
    pinMode(PIN_AUX5,OUTPUT);  //RADAR
}

MARCDUINO_ACTION(Periscopio, *P001, ({
    printf("--> PERISCOPIO\n");
    digitalWrite(PIN_AUX1, HIGH);
    delay (500);
    digitalWrite(PIN_AUX1, LOW);
}))

MARCDUINO_ACTION(BadMotivador, *BM001, ({
    printf("--> BAD MOTIVADOR\n");
    digitalWrite(PIN_AUX2, HIGH);
    delay (500);
    digitalWrite(PIN_AUX2, LOW);
}))


MARCDUINO_ACTION(Zapper, *ZP001, ({
    printf("--> Zapper\n");
    digitalWrite(PIN_AUX3, HIGH);
    delay (500);
    digitalWrite(PIN_AUX3, LOW);
}))

MARCDUINO_ACTION(SableLuz, *SL001, ({
    printf("--> Sable Luz\n");
    digitalWrite(PIN_AUX4, HIGH);
    delay (500);
    digitalWrite(PIN_AUX5, LOW);
}))

MARCDUINO_ACTION(Radar, *RD001, ({
    printf("--> RADAR\n");
    digitalWrite(PIN_AUX5, HIGH);
    delay (500);
    digitalWrite(PIN_AUX5, LOW);
}))


MARCDUINO_ACTION(TodosArriba, TDALL, ({
    printf("--> TODOS!!!\n");
    digitalWrite(PIN_AUX1, HIGH);
    digitalWrite(PIN_AUX2, HIGH);
    digitalWrite(PIN_AUX3, HIGH);
    digitalWrite(PIN_AUX4, HIGH);
    digitalWrite(PIN_AUX5, HIGH);
    delay (500);
    digitalWrite(PIN_AUX1, LOW);
    digitalWrite(PIN_AUX2, LOW);
    digitalWrite(PIN_AUX3, LOW);
    digitalWrite(PIN_AUX4, LOW);
    digitalWrite(PIN_AUX5, LOW);
}))
////////////////
