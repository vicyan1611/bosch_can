# webserver/firebase_client.py
import firebase_admin
from firebase_admin import credentials, firestore
import uuid
import os
from datetime import datetime, timezone

class FirebaseClient:
    def __init__(self, credential_path, device_id_file='device_id.txt'):
        try:
            # Check if Firebase app is already initialized
            if not firebase_admin._apps:
                cred = credentials.Certificate(credential_path)
                firebase_admin.initialize_app(cred)
            
            self.db = firestore.client()
            self.device_id = self._get_or_create_device_id(device_id_file)
            self.display_name = self._get_display_name()
            print(f"Firebase Client initialized for device: {self.device_id} ({self.display_name})")

        except Exception as e:
            print(f"Error initializing Firebase: {e}")
            print("Firebase features will be disabled. Please check your credentials.")
            self.db = None
            self.device_id = 'offline-device'
            self.display_name = 'Offline User'

    def _get_or_create_device_id(self, file_path):
        """Gets the device ID from a local file, or creates a new one."""
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                return f.read().strip()
        else:
            new_id = str(uuid.uuid4())
            with open(file_path, 'w') as f:
                f.write(new_id)
            return new_id

    def _get_display_name(self):
        """Gets the display name from Firestore, or creates a default one."""
        if not self.db:
            return "Offline User"
            
        device_ref = self.db.collection('devices').document(self.device_id)
        doc = device_ref.get()
        if doc.exists:
            return doc.to_dict().get('displayName', 'New Driver')
        else:
            # Create a default entry for this new device
            default_name = f"Driver-{self.device_id[:4]}"
            device_ref.set({'displayName': default_name})
            return default_name

    def get_device_info(self):
        """Returns a dictionary with device ID and display name."""
        return {
            "deviceId": self.device_id,
            "displayName": self.display_name
        }

    def save_trip(self, score, summary):
        """Saves a trip's data to the 'trips' collection and updates the leaderboard."""
        if not self.db:
            print("Cannot save trip, Firebase is not connected.")
            return None # Return None to indicate failure

        timestamp = datetime.now(timezone.utc).isoformat()
        
        trip_data = {
            'deviceId': self.device_id,
            'score': score,
            'summary': summary,
            'timestamp': timestamp,
            'llmAdvice': 'Pending...' # To be filled by the Cloud Function
        }
        
        try:
            # Add to trips collection
            update_time, trip_ref = self.db.collection('trips').add(trip_data)
            print(f"Trip saved with ID: {trip_ref.id}")

            # Update leaderboard
            self._update_leaderboard(score)
            
            return trip_ref.id # Return the new trip ID
        except Exception as e:
            print(f"Error saving trip to Firestore: {e}")
            return None

    def _update_leaderboard(self, current_score):
        """Updates the user's best score on the leaderboard if the new score is higher."""
        if not self.db:
            return

        leaderboard_ref = self.db.collection('leaderboard').document(self.device_id)
        
        try:
            doc = leaderboard_ref.get()
            if doc.exists:
                best_score = doc.to_dict().get('bestScore', 0)
                if current_score > best_score:
                    leaderboard_ref.update({
                        'bestScore': current_score,
                        'displayName': self.display_name # Also update name in case it changed
                    })
                    print(f"New best score for {self.display_name}: {current_score}")
            else:
                # First time on the leaderboard for this device
                leaderboard_ref.set({
                    'bestScore': current_score,
                    'displayName': self.display_name
                })
                print(f"Added {self.display_name} to leaderboard with score: {current_score}")
        except Exception as e:
            print(f"Error updating leaderboard: {e}")

    def get_leaderboard(self, limit=10):
        """Fetches the top drivers from the leaderboard."""
        if not self.db:
            return []
        
        try:
            # Firestore's 'order_by' requires a corresponding index. 
            # You'll need to create this in the Firebase console.
            # Go to Firestore -> Indexes -> Add Indexing
            # Collection ID: leaderboard, Field: bestScore (Descending), Add any other fields you need.
            query = self.db.collection('leaderboard').order_by(
                'bestScore', direction=firestore.Query.DESCENDING
            ).limit(limit)
            
            results = query.stream()
            leaderboard = []
            for doc in results:
                data = doc.to_dict()
                data['deviceId'] = doc.id
                leaderboard.append(data)
            return leaderboard
        except Exception as e:
            print(f"Error fetching leaderboard: {e}")
            print("Please ensure you have created the necessary index in Firestore.")
            return []

    def get_trip_advice(self, trip_id):
        """Fetches a specific trip document to check for LLM advice."""
        if not self.db:
            return None
        try:
            trip_ref = self.db.collection('trips').document(trip_id)
            doc = trip_ref.get()
            if doc.exists:
                return doc.to_dict()
            else:
                return None
        except Exception as e:
            print(f"Error fetching trip advice for {trip_id}: {e}")
            return None

