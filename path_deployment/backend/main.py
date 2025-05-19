import time
import json
from flask import Flask, request, render_template, jsonify,  Response

from gnss_read import GNSSData 
from generate_path import Path_plan
from run import Run_process

app = Flask(__name__)
gcp=[]

#data_read = GNSSData("/home/robo/dev/fil_/path_deployment/src/canbus/serial_posix_v2/inc/gnr.buf")
data_read = GNSSData("/home/ubuntu/serial_posix_v2.0/inc/gnr.buf")
#proc = Run_process("../src/","gnss")

def event_stream():
    while True:
        try:
            data = data_read.last_data()
            if data:
                yield f"data: {json.dumps(data)}\n\n"
            else:
                yield "data: {}\n\n"
            #time.sleep(0.5)
        except Exception as e:
            print(f"Error in stream: {e}")
            break
                
@app.route("/")
def root():
    return render_template("page1.html")

@app.route('/foo', methods=['POST']) 
def foo():

    gcp.clear()
    data = request.json 
    f = open('../data/data_log.txt', 'w')
    for item in data:
       f.write("%s\n" % item)
    f.close()

    outt=[]
    for n in range(0,len(data[0])):
        outt=[[data[len(data)-1][n-1]["lat"],data[len(data)-1][n-1]["lng"]],[data[len(data)-1][n]["lat"],data[len(data)-1][n]["lng"]]]
        gcp.append(outt)

    return jsonify(data)

@app.route('/path', methods=['GET']) 
def path():
    infor={}
    out= Path_plan(gcp)
    tp=out.path()
    print("tp: ",tp)

    infor["track"]=tp[:24]

    return jsonify(infor)

@app.route('/livelocation', methods=['GET'])
def livelocation():
    #proc.runobject()
    #time.sleep(1)
    return Response(event_stream(), mimetype="text/event-stream")

@app.route('/stop', methods=['GET'])
def stop():
    #response = proc.stopobject()
    #res = {"response":response}
    res = {"response":"stopped"}
    return jsonify(res)
    
    

if __name__ == '__main__':
    try:
        app.run(host="0.0.0.0", port=3000, threaded=True)
    except:
        pass
    '''except KeyboardInterrupt:
        response = proc.stopobject()
    except Exception as e:
        response = proc.stopobject()'''
