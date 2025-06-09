import serial

# Configure the serial port
ser = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)

try:
    while True:
#         print("Cant fetch data inside")
        # Read a line of NMEA data from the serial port
        nmea_sentence = ser.readline().decode('utf-8').strip()
        print(nmea_sentence)
        
        # Check if the sentence is a GGA (Global Positioning System Fix Data) sentence
        if nmea_sentence.startswith('$GPGGA'):
            # Split the sentence into fields
            fields = nmea_sentence.split(',')
            # Extract latitude, longitude, and other relevant data
            if len(fields) >= 10:
                # Latitude in format ddmm.mmmm
                latitude = fields[2]
                # Longitude in format dddmm.mmmm
                longitude = fields[4]
                # Latitude hemisphere (N or S)
                lat_hemisphere = fields[3]
                # Longitude hemisphere (E or W)
                lon_hemisphere = fields[5]
                
                # Convert latitude and longitude to decimal format
                latitude_dec = float(latitude[:2]) + float(latitude[2:]) / 60
                longitude_dec = float(longitude[:3]) + float(longitude[3:]) / 60
                
                # Adjust latitude and longitude based on hemisphere
                if lat_hemisphere == 'S':
                    latitude_dec *= -1
                if lon_hemisphere == 'W':
                    longitude_dec *= -1
                
                print(f"Latitude: {latitude_dec}, Longitude: {longitude_dec}")
                print("NMEA Sentence:", nmea_sentence)
                
except KeyboardInterrupt:
    ser.close()
    print("GPS script stopped")
