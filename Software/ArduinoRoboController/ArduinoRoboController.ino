/*
 * Basic Arduino-based robot HMI.
 * By Tennessee Carmel-Veilleux.
 *
 * 5 analog axes and 1 digital button. All samples emitted at every sampling interval.
 * Reduced-reporting implemented by client software on PC side.
 * Simple 1 pole digital exponential averager used for filtering analog samples
 */

const int BUTTON_PIN = 12;

// True if system is processing first sample
bool first_sample = true;
// Exponential averager history (y[n-1])
long int history[5];
// Exponential averager factor in Q10.8 format
const long int alpha_factor = 230; // Approx 0.9

const int channels[5] = { A0, A1, A2, A3, A4 };

void setup() {
    // initialize serial communication at 9600 bits per second:
    Serial.begin(115200);  
   
    // Use internal pull-up on button digital input
    pinMode(BUTTON_PIN, INPUT_PULLUP);
}

// the loop routine runs over and over again forever:
void loop() {
    long int values[5];
  
    for (unsigned int chan_idx = 0; chan_idx < 5; chan_idx++) {
        values[chan_idx] = analogRead(channels[chan_idx]);
        // Prime filters on first sample
        if (first_sample) {
            history[chan_idx] = values[chan_idx];
        }
        
        // Apply exponential averager. Math is in Q10.8 format
        values[chan_idx] = (((history[chan_idx] << 8) * (256 - alpha_factor)) + ((values[chan_idx] << 8) * alpha_factor)) >> 16; 
        
        // Save sample in history
        history[chan_idx] = values[chan_idx];
    }
  
    int button = !digitalRead(BUTTON_PIN);
    
    // Print-out all values of this sampling cycle
    Serial.print("CTRL,");
    Serial.print(values[0]);
    Serial.print(",");
    Serial.print(values[1]);
    Serial.print(",");
    Serial.print(values[2]);
    Serial.print(",");
    Serial.print(values[3]);
    Serial.print(",");
    Serial.print(values[4]);
    Serial.print(",");
    Serial.println(button);
    
    // Delay in between reads to throttle the system
    delay(50);
    
    first_sample = false;
}
