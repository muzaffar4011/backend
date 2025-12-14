import psycopg2

# Your Neon connection string
conn_string = "postgresql://neondb_owner:npg_8mHhd6OpUAjN@ep-curly-voice-a4yn2qyo-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require"

try:
    print("Connecting to Neon...")
    conn = psycopg2.connect(conn_string)
    print("✅ Connected successfully!")
    
    cursor = conn.cursor()
    cursor.execute("SELECT version();")
    version = cursor.fetchone()
    print(f"PostgreSQL Version: {version[0]}")
    
    conn.close()
    print("✅ Connection closed. Ready to use!")
except Exception as e:
    print(f"❌ Connection failed: {e}")