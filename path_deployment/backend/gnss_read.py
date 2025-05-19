import os

class GNSSData:
    def __init__(self, filepath):
        """
        :param filepath - path to data file
        """
        self.filepath = filepath
        self._last_line = None

    def last_data(self):
        if not os.path.exists(self.filepath):
            print("The specified path does not exist")
            return None

        try:
            with open(self.filepath, 'rb') as file:
                file.seek(0, os.SEEK_END)
                file_size = file.tell()
                block_size = 1024
                data = b''
                while file_size > 0:
                    seek_size = min(block_size, file_size)
                    file.seek(file_size - seek_size)
                    chunk = file.read(seek_size)
                    data = chunk + data
                    file_size -= seek_size
                    if data.count(b'\n') >= 2:
                        break

                lines = data.split(b'\n')
                
                if len(lines) >= 2:
                    second_last = lines[-2] if lines[-1] == b'' else lines[-3]
                    line = second_last.decode().strip()

                    if line == self._last_line:
                        return None  

                    self._last_line = line
                    # pattern :  speed:0.04 Lat:28.432915 Lng:77.306671 head:0.00 hdop:0.99 height:214.40 LatK:28.432915 LngK:77.306671
                    if "Lat" in line and "Lng" in line:
                        parts = line.split(" ")
                        #print(parts)
                        lat = float(parts[1].split(":")[1])
                        lon = float(parts[2].split(":")[1])
                        #latk = float(parts[6].split(":")[1])
                        #lonk = float(parts[7].split(":")[1])
                        
                        #return {"latitude": lat, "longitude": lon, "latk":latk, "lonk":lonk}
                        return {"latitude": lat, "longitude": lon}

        except Exception as e:
            print(f"Error reading GPS data: {e}")

        return None

