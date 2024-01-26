/*Preferences:

SavePeers: 
Id      (Vergabe bei Pairing)
Type    
Name
Periphs (ID, Type, Name)

Peers (0 bis MAX_PEERS)
    P0-Id     - P[0].Id  
    P0-Type   - P[0].Type         
    P0-Name   - P[0].Name                 
    P0-MAC    - P[0].MAC          
  
    Periph(0 bis MAX_PERIPHERALS)
        P0-Periph0.Id
        P0-Periph0.Type
        P0-Periph0.Name

MultiScreens: (0 bis MULTI_SCREENS)
    S0-Name   - Screen[0].Name    
    S0-Id     - Screen[0].Id

    Periph (0 bis PERIPH_PER_SCREEN)
        S0-PeriphId0      - Screen[0].PeriphId[0]
        S0-PeerId0        - Screen[0].PeerId[0]


GetPeers: 
Id      
Type    
Name
Periphs (ID, Type, Name)

aufgefüllt wird 
    PNumber
    ??? Peer:   aktueller Pointer der Periphs (P[PNr].Periph[SNr]) aus PeriphId[SNr]
    Periph: Parent-Peer-Id (P[PNr].Periph[SNr].PeerId)
    Screen[].Periph[] Pointer aus PeriphId[]
    
Peers (0 bis MAX_PEERS)
    P0-Id     - P[0].Id  
    P0-Type   - P[0].Type         
    P0-Name   - P[0].Name                 
    P0-MAC    - P[0].MAC          
  
    Periph(0 bis MAX_PERIPHERALS)
        P0-Periph0.Id
        P0-Periph0.Type
        P0-Periph0.Name

MultiScreens: (0 bis MULTI_SCREENS)
    S0-Id     - Screen[0].Id
    S0-Name   - Screen[0].Name    
    
    Periph (0 bis PERIPH_PER_SCREEN)
        S0-PeerId0        - Screen[0].PeerId[0]
        S0-PeriphId0      - Screen[0].PeriphId[0]
    

        
    
*/