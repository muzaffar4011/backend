import psycopg2

# Your Neon connection string
conn_string = "postgresql://neondb_owner:npg_8mHhd6OpUAjN@ep-curly-voice-a4yn2qyo-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require"

try:
    print("Connecting to Neon...")
    conn = psycopg2.connect(conn_string)
    cursor = conn.cursor()
    
    cursor.execute("""
        SELECT table_name 
        FROM information_schema.tables 
        WHERE table_schema = 'public'
        ORDER BY table_name;
    """)
    
    tables = cursor.fetchall()
    print("\n✅ Tables in database:")
    for table in tables:
        print(f"  - {table[0]}")
    
    expected_tables = ['alembic_version', 'chat_sessions', 'user_queries', 'chat_responses', 'user_feedback']
    table_names = [t[0] for t in tables]
    
    if len(tables) == 5:
        print(f"\n✅ All {len(tables)} tables created successfully!")
        print("\n✅ Migration completed successfully!")
    else:
        print(f"\n⚠️  Expected 5 tables, found {len(tables)}")
        missing = [t for t in expected_tables if t not in table_names]
        if missing:
            print(f"⚠️  Missing tables: {missing}")
    
    conn.close()
    print("\n✅ Database verification complete!")
    
except Exception as e:
    print(f"❌ Error: {e}")
