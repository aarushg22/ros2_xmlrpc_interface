
# method to call
string method

#Data required for create and update task functions
uint16 user_id
string pickup
string delivery
string[] payloads
uint8 type
string start_time
uint16 delivery_end

# agv_id required for any agv_related query
uint8 agv_id

# request_id required for any cancellation of request related query
uint16 req_id

#to get request states, the request id list
uint16[] requests
