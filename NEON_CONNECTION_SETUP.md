# Neon Connection String Setup - Quick Guide

Aapko jo connection string mila hai, usko ab setup karna hai.

---

## ✅ Your Connection String

```
postgresql://neondb_owner:npg_8mHhd6OpUAjN@ep-curly-voice-a4yn2qyo-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require
```

**Good News**: Ye **pooled connection string** hai (contains `-pooler`) - perfect for Render! ✅

---

## Step 1: Test Connection Locally (Optional but Recommended)

### Option A: Using Python Script

1. Create file `test_neon_connection.py` in `backend` folder:
```python
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
```

2. Run test:
```bash
cd backend
python test_neon_connection.py
```

**Expected Output:**
```
Connecting to Neon...
✅ Connected successfully!
PostgreSQL Version: PostgreSQL 16.x...
✅ Connection closed. Ready to use!
```

### Option B: Using Neon SQL Editor

1. Go to Neon Dashboard → Your Project
2. Click **"SQL Editor"** (left sidebar)
3. Run:
```sql
SELECT version();
```
4. Should return PostgreSQL version

---

## Step 2: Add to Render Environment Variables

### 2.1 Go to Render Dashboard

1. Open: https://dashboard.render.com/
2. Login to your account
3. Find your service: `rag-chatbot-api` (ya jo name aapne diya)
4. Click on service name

### 2.2 Add DATABASE_URL

1. Click **"Environment"** tab (left sidebar)
2. Click **"Add Secret"** or **"Add Environment Variable"** button
3. Fill in:
   ```
   Key: DATABASE_URL
   Value: postgresql://neondb_owner:npg_8mHhd6OpUAjN@ep-curly-voice-a4yn2qyo-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require
   ```
4. Click **"Save Changes"**

**Important**: 
- Value field me **complete connection string** paste karna hai
- Double-check - no extra spaces
- Password included hai - secure rakho

---

## Step 3: Run Database Migration

Ab database tables create karni hain.

### Option A: Using Render Shell (Recommended)

1. Render Dashboard → Your Service → **"Shell"** tab
2. Run:
```bash
cd backend
alembic upgrade head
```

**Expected Output:**
```
INFO  [alembic.runtime.migration] Context impl PostgresqlImpl.
INFO  [alembic.runtime.migration] Will assume transactional DDL.
INFO  [alembic.runtime.migration] Running upgrade  -> 9d1a4c9ba618, initial migration
```

### Option B: Run Locally

1. Set environment variable:
```bash
# Windows PowerShell
$env:DATABASE_URL="postgresql://neondb_owner:npg_8mHhd6OpUAjN@ep-curly-voice-a4yn2qyo-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require"

# Windows CMD
set DATABASE_URL=postgresql://neondb_owner:npg_8mHhd6OpUAjN@ep-curly-voice-a4yn2qyo-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require
```

2. Run migration:
```bash
cd backend
alembic upgrade head
```

---

## Step 4: Verify Tables Created

### Using Neon SQL Editor

1. Neon Dashboard → Your Project → **SQL Editor**
2. Run:
```sql
SELECT table_name 
FROM information_schema.tables 
WHERE table_schema = 'public'
ORDER BY table_name;
```

3. **Expected Tables:**
   - `alembic_version`
   - `chat_sessions`
   - `user_queries`
   - `chat_responses`
   - `user_feedback`

### Using Python Script

Create `verify_tables.py`:
```python
import psycopg2

conn_string = "postgresql://neondb_owner:npg_8mHhd6OpUAjN@ep-curly-voice-a4yn2qyo-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require"

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

if len(tables) == 5:
    print("\n✅ All tables created successfully!")
else:
    print(f"\n⚠️  Expected 5 tables, found {len(tables)}")

conn.close()
```

Run:
```bash
cd backend
python verify_tables.py
```

---

## Step 5: Test Render Service

After migration, Render service should connect to database automatically.

### 5.1 Check Render Logs

1. Render Dashboard → Your Service → **"Logs"** tab
2. Look for:
   - ✅ "Application startup complete"
   - ✅ No database connection errors

### 5.2 Test Health Endpoint

```bash
curl https://your-service-name.onrender.com/api/health
```

Should return:
```json
{
  "status": "healthy",
  "timestamp": "..."
}
```

### 5.3 Test Session Creation

```bash
curl -X POST https://your-service-name.onrender.com/api/session \
  -H "Content-Type: application/json"
```

Should create a session successfully.

---

## Troubleshooting

### ❌ Connection Failed in Render

**Problem**: Render service can't connect to Neon

**Check:**
1. `DATABASE_URL` environment variable correctly set?
2. Connection string copied completely? (No spaces/truncation)
3. Render service redeployed after adding env var?

**Solution**: 
- Environment variable add karne ke baad service **restart** karo (Manual Deploy → Deploy latest commit)

### ❌ Migration Fails

**Problem**: `alembic upgrade head` fails

**Check:**
1. Connection string correct?
2. Database accessible?
3. Environment variable set properly?

**Solution**:
```bash
# Test connection first
python test_neon_connection.py

# Then run migration
alembic upgrade head
```

### ❌ Tables Not Created

**Problem**: Migration runs but tables missing

**Solution**:
1. Check Neon SQL Editor - any error messages?
2. Verify migration actually completed
3. Re-run migration if needed:
```bash
alembic upgrade head
```

---

## Security Reminder

⚠️ **Important**: Connection string me password included hai!

**Never:**
- ❌ Commit to Git
- ❌ Share publicly
- ❌ Post on forums/chat

**Only Store In:**
- ✅ Render environment variables (encrypted)
- ✅ Local `.env` file (already in `.gitignore`)
- ✅ Password manager

---

## Quick Checklist

- [ ] Connection string test kiya (local test script)
- [ ] `DATABASE_URL` added to Render environment variables
- [ ] Render service redeployed (environment variable changes ke baad)
- [ ] Migration run successfully (`alembic upgrade head`)
- [ ] Tables verified (5 tables created)
- [ ] Render service logs check kiye (no database errors)
- [ ] API health check successful

---

## Next Steps

After database setup complete:

1. ✅ **Content Ingest**: Qdrant me content upload karo
   ```bash
   python scripts/ingest_content.py --source ../book_frontend/docs --create-collection --force
   ```

2. ✅ **Test API**: Chat endpoint test karo
   ```bash
   curl -X POST https://your-service.onrender.com/api/chat \
     -H "Content-Type: application/json" \
     -d '{"query_text": "What is ROS 2?", "mode": "full_book"}'
   ```

3. ✅ **Frontend Update**: Frontend me production API URL update karo

---

**Database Setup Complete!** 🎉

Neon database connected aur ready hai Render deployment ke liye!
