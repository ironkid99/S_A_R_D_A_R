# import cv2
# import face_recognition
# 
# # Load the reference image from local storage
# reference_image = face_recognition.load_image_file("rr.jpg")
# reference_face_encoding = face_recognition.face_encodings(reference_image)[0]
# 
# # Start capturing frames from the camera
# cap = cv2.VideoCapture(0)  # Change to 1 if you have multiple cameras, or provide the path to a video file
# 
#  while True:
#     ret, frame = cap.read()
#     if not ret:
#         break
# 
#     # Resize the frame for faster processing
#     frame = cv2.resize(frame, (600, 400))
# 
#     # Find all face locations and face encodings in the current frame
#     face_locations = face_recognition.face_locations(frame)
#     face_encodings = face_recognition.face_encodings(frame, face_locations)
# 
#     # Compare each detected face with the reference face
#     for face_encoding in face_encodings:
#         # Compare the current face encoding with the reference face encoding
#         match = face_recognition.compare_faces([reference_face_encoding], face_encoding)
#         
#         # If a match is found, display a message
#         if match[0]:
#             cv2.putText(frame, "Criminal Detected", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
# 
#     # Display the frame
#     cv2.imshow('Real-time Face Recognition', frame)
# 
#     # Press 'q' to quit
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
# 
# cap.release()
# cv2.destroyAllWindows()
#


import cv2
import face_recognition

reference_images = ["tf.jpg"]
reference_face_encodings = []

for image_path in reference_images:
    reference_image = face_recognition.load_image_file(image_path)
    
    face_locations = face_recognition.face_locations(reference_image)
    if len(face_locations) > 0:
        
        reference_face_encoding = face_recognition.face_encodings(reference_image)[0]
        reference_face_encodings.append(reference_face_encoding)
    else:
        print(f"No face detected in {image_path}")


#cap = cv2.VideoCapture(1)
cap = cv2.VideoCapture(0)  

while True:
    ret, frame = cap.read()
    if not ret:
        break

    
    frame = cv2.resize(frame, (600, 400))

   
    face_locations = face_recognition.face_locations(frame)
    face_encodings = face_recognition.face_encodings(frame, face_locations)

 
    for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):

        match_status = [False] * len(reference_images)

      
        for i, reference_face_encoding in enumerate(reference_face_encodings):
            match_status[i] = face_recognition.compare_faces([reference_face_encoding], face_encoding)[0]


        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

        if any(match_status):
            cv2.putText(frame, "Intruder Detected", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

  
    cv2.imshow('Real-time Face Recognition', frame)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
