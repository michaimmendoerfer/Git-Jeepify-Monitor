void SavePeers() {
  Serial.println("SavePeers...");
  preferences.begin("JeepifyPeers", false);
  
  char Buf[50] = {}; String BufS;

  PeerCount = -1;

  for (int Pi=0; Pi< MAX_PEERS; Pi++) {
    if (P[Pi].Type > 0) {
      PeerCount++;
      //P.Type...
      sprintf(Buf, "P%d-Type", Pi);
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(P[Pi].Type);
      if (preferences.getInt(Buf, 0) != P[Pi].Type) preferences.putInt(Buf, P[Pi].Type);
      
      //P.Id
      sprintf(Buf, "P%d-Id", Pi);
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(P[Pi].Id);
      if (preferences.getInt(Buf, 0) != P[Pi].Id) preferences.putInt(Buf, P[Pi].Id);
      
      //P.BroadcastAddress
      sprintf(Buf, "P%d-MAC", Pi);
      preferences.putBytes(Buf, P[Pi].BroadcastAddress, 6);
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); PrintMAC(P[Pi].BroadcastAddress); Serial.println();
      
      //P.Name
      sprintf(Buf, "P%d-Name", Pi);
      BufS = P[Pi].Name;
      Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(BufS);
      if (preferences.getString(Buf, "") != BufS) preferences.putString(Buf, BufS);

      for (int Si=0; Si<MAX_PERIPHERALS; Si++) {
        sprintf(Buf, "P%d-Periph%d-Name", Pi, Si);
        BufS = P[Pi].S[Si].Name;
        Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(P[Pi].S[Si].Name);
        if (preferences.getString(Buf, "") != BufS) preferences.putString(Buf, BufS);
        
        sprintf(Buf, "P%d-Periph%d-Type", Pi, Si);
        Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(P[Pi].S[Si].Type);
        if (preferences.getInt(Buf, 0) != P[Pi].S[Si].Type) preferences.putInt(Buf, P[Pi].S[Si].Type);

        sprintf(Buf, "P%d-Periph%d-Id", Pi, Si);
        Serial.print("schreibe "); Serial.print(Buf); Serial.print(" = "); Serial.println(P[Pi].S[Si].Id);
        if (preferences.getInt(Buf, 0) != P[Pi].S[Si].Id) preferences.putInt(Buf, P[Pi].S[Si].Id);
      }
    }
  }
  for (int s=0; s<MULTI_SCREENS; s++) {
    if !((Screen[s].Name != "") or Screen[s].Name = NULL)) {
      sprintf(Buf, "S%d-Name", s);
      if (preferences.getString(Buf,"") != Screen[s].Name)   preferences.putString(Buf, Screen[s].Name);
      sprintf(Buf, "S%d-PeerId", s);
      if (preferences.getInt(Buf,0) != Screen[s].PeerId) preferences.putInt(Buf, Screen[s].PeerId); 
      sprintf(Buf, "S%d-Id", s);
      if (preferences.getInt(Buf,0) != Screen[s].Id)     preferences.putInt(Buf, Screen[s].Id);      
  
      for (int p=0; p<PERIPH_PER_SCREEN; p++) {
          if (Screen[s].PeriphId[p]) {
              sprintf(Buf, "S%d-PeriphId%d", s, p);
              if (preferences.getInt(Buf,0) != Screen[s].PeriphId[p]) preferences.putInt(Buf, Screen[s].PeriphId[p]);
          }
      }
    }
  }
  if (preferences.getInt("PeerCount") != PeerCount) preferences.putInt("PeerCount", PeerCount);
  
  preferences.end();
}