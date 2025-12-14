# Neon PostgreSQL Setup Guide

Neon is a serverless PostgreSQL database with a free tier. This guide will help you set it up for the RAG Chatbot backend.

---

## Step 1: Create Neon Account

1. **Go to Neon Website**: https://neon.tech/
2. **Sign Up**:
   - Click "Sign Up" (top right)
   - Choose GitHub, Google, or email sign up
   - Verify your email if needed

---

## Step 2: Create a New Project

1. **After Login**: You'll be on the Neon dashboard
2. **Create Project**:
   - Click **"Create Project"** button
   - **Project Name**: `rag-chatbot-db` (or any name you prefer)
   - **Region**: Choose closest to you (e.g., US East, US West, EU)
   - **PostgreSQL Version**: Default (15 or 16) - both work fine
3. **Click "Create Project"**
4. Wait ~30 seconds for database to be created

---

## Step 3: Get Connection String

### 3.1 Find Connection Details

Once project is created, you'll see:

1. **Dashboard** shows your project
2. Click on your project name
3. You'll see connection details in the dashboard

### 3.2 Connection String Types

Neon provides two types of connection strings:

#### **Option A: Pooled Connection (Recommended for Render)**

- **Format**: Contains `-pooler` in the URL
- **Example**: 
  ```
  postgresql://username:password@ep-xxx-xxx-pooler.us-east-2.aws.neon.tech/dbname?sslmode=require
  ```
- **Benefits**: 
  - Better for serverless/serverless-like environments
  - Handles connection pooling automatically
  - Recommended for Render deployment

#### **Option B: Direct Connection**

- **Format**: Does NOT contain `pooler`
- **Example**:
  ```
  postgresql://username:password@ep-xxx-xxx.us-east-2.aws.neon.tech/dbname?sslmode=require
  ```
- **Use**: For local development or direct connections

### 3.3 Copy Connection String

1. In Neon dashboard, look for **"Connection Details"** or **"Connection String"** section
2. Click **"Pooled connection"** or **"Connection pooling"** tab
3. Copy the **full connection string** (starts with `postgresql://...`)
4. **Save this somewhere safe** - you'll need it for Render environment variables

**Important**: The connection string includes password - keep it secure!

---

## Step 4: Test Connection (Optional)

### Option A: Using Neon SQL Editor

1. In Neon dashboard, click **"SQL Editor"** (left sidebar)
2. Run a test query:
   ```sql
   SELECT version();
   ```
3. Should return PostgreSQL version

### Option B: Using Local psql (If installed)

```bash
psql "your-connection-string-here"
```

Then run:
```sql
SELECT version();
\q  -- to quit
```

### Option C: Using Python Script

Create a test file `test_neon.py`:
```python
import psycopg2

# Replace with your connection string
conn_string = "postgresql://user:password@host/dbname?sslmode=require"

try:
    conn = psycopg2.connect(conn_string)
    print("✅ Connected to Neon successfully!")
    cursor = conn.cursor()
    cursor.execute("SELECT version();")
    print(cursor.fetchone())
    conn.close()
except Exception as e:
    print(f"❌ Connection failed: {e}")
```

Run:
```bash
python test_neon.py
```

---

## Step 5: Configure for Render Deployment

### 5.1 Use Pooled Connection String

For Render deployment, **always use the pooled connection string** because:
- Render services can have connection limits
- Pooling handles multiple connections better
- Prevents connection exhaustion errors

### 5.2 Add to Render Environment Variables

1. Go to **Render Dashboard** → Your Web Service
2. Go to **Environment** tab
3. Click **"Add Secret"** or **"Add Environment Variable"**
4. Add:
   ```
   Key: DATABASE_URL
   Value: [paste your pooled connection string from Neon]
   ```
5. Click **"Save Changes"**

---

## Step 6: Database Migration

After setting up Neon and adding to Render:

### Option A: Run Migration via Render Shell

1. Render Dashboard → Your Service → **Shell** tab
2. Run:
   ```bash
   cd backend
   alembic upgrade head
   ```

### Option B: Run Migration Locally

1. Set environment variable:
   ```bash
   # Windows PowerShell
   $env:DATABASE_URL="your-neon-connection-string"
   
   # Windows CMD
   set DATABASE_URL=your-neon-connection-string
   
   # Mac/Linux
   export DATABASE_URL="your-neon-connection-string"
   ```

2. Run migration:
   ```bash
   cd backend
   alembic upgrade head
   ```

### Expected Output:
```
INFO  [alembic.runtime.migration] Context impl PostgresqlImpl.
INFO  [alembic.runtime.migration] Will assume transactional DDL.
INFO  [alembic.runtime.migration] Running upgrade  -> 9d1a4c9ba618, initial migration
```

---

## Step 7: Verify Tables Created

### Using Neon SQL Editor:

1. Go to Neon Dashboard → Your Project → **SQL Editor**
2. Run:
   ```sql
   SELECT table_name 
   FROM information_schema.tables 
   WHERE table_schema = 'public';
   ```
3. Should show these tables:
   - `alembic_version`
   - `chat_sessions`
   - `user_queries`
   - `chat_responses`
   - `user_feedback`

### Using Python Script:

```python
import psycopg2

conn_string = "your-connection-string"
conn = psycopg2.connect(conn_string)
cursor = conn.cursor()

cursor.execute("""
    SELECT table_name 
    FROM information_schema.tables 
    WHERE table_schema = 'public'
    ORDER BY table_name;
""")

tables = cursor.fetchall()
print("Tables in database:")
for table in tables:
    print(f"  - {table[0]}")

conn.close()
```

---

## Important Notes

### Connection String Format

**Pooled (Recommended)**:
```
postgresql://username:password@ep-xxx-xxx-pooler.region.aws.neon.tech/dbname?sslmode=require
```

**Direct**:
```
postgresql://username:password@ep-xxx-xxx.region.aws.neon.tech/dbname?sslmode=require
```

### SSL Mode

- Neon requires SSL connections
- Connection string automatically includes `?sslmode=require`
- If you build connection string manually, always add `?sslmode=require`

### Password in Connection String

- Connection string includes password
- **Never commit to Git**
- Only store in:
  - Render environment variables
  - Local `.env` file (in `.gitignore`)
  - Password managers

### Free Tier Limits

Neon Free Tier includes:
- ✅ **0.5 GB** storage (enough for this project)
- ✅ **512 MB** compute time (auto-scales)
- ✅ **Branching** support (useful for dev/staging)
- ✅ **Connection pooling** included
- ✅ **Automatic backups**

Limitations:
- ⚠️ Compute time limited (auto-pauses when not in use)
- ⚠️ Storage limit (can upgrade if needed)

---

## Troubleshooting

### ❌ Connection Failed: SSL Required

**Problem**: `FATAL: SSL connection is required`

**Solution**: 
- Make sure connection string includes `?sslmode=require`
- Check you're using the correct connection string

### ❌ Too Many Connections

**Problem**: `FATAL: remaining connection slots are reserved`

**Solution**:
- Use **pooled connection string** (contains `-pooler`)
- Check connection pooling is enabled
- Close unused connections properly in code

### ❌ Connection Timeout

**Problem**: Connection times out

**Solution**:
- Check network/firewall settings
- Verify connection string is correct
- Try direct connection instead of pooled (or vice versa)
- Check Neon dashboard - is project active?

### ❌ Authentication Failed

**Problem**: `FATAL: password authentication failed`

**Solution**:
- Verify connection string password is correct
- Copy connection string again from Neon dashboard
- Check for extra spaces or special characters
- Reset password in Neon if needed

### ❌ Database Doesn't Exist

**Problem**: `FATAL: database "xyz" does not exist`

**Solution**:
- Check database name in connection string
- Default Neon database is usually `neondb` or `main`
- Verify in Neon dashboard → Connection Details

---

## Security Best Practices

1. **Never commit connection strings to Git**
   - Use `.env` files (already in `.gitignore`)
   - Use environment variables in Render

2. **Use Pooled Connections**
   - Better for serverless environments
   - Prevents connection exhaustion

3. **Rotate Passwords Regularly**
   - Neon dashboard → Settings → Reset password

4. **Use SSL Always**
   - Neon requires it anyway
- Always include `?sslmode=require`

5. **Limit Database Access**
   - Only give access to services that need it
   - Use separate databases for dev/prod

---

## Quick Checklist

- [ ] Neon account created
- [ ] Project created
- [ ] Pooled connection string copied
- [ ] Connection string added to Render environment variables
- [ ] Database migration run successfully
- [ ] Tables verified in Neon SQL Editor
- [ ] Test connection successful

---

## Useful Neon Features

### Branching
- Create database branches for testing
- Similar to Git branching
- Useful for dev/staging environments

### SQL Editor
- Built-in SQL editor in Neon dashboard
- No need for external tools
- Great for quick queries

### Monitoring
- Query performance metrics
- Connection monitoring
- Resource usage stats

### Automatic Backups
- Point-in-time recovery
- Automatic daily backups
- Restore from any point

---

## Next Steps

After Neon setup:

1. ✅ **Add to Render**: Set `DATABASE_URL` environment variable
2. ✅ **Run Migrations**: `alembic upgrade head`
3. ✅ **Test Connection**: Verify tables created
4. ✅ **Deploy Backend**: Render deployment should work
5. ✅ **Test API**: Verify database operations work

---

## Support

- **Neon Docs**: https://neon.tech/docs
- **Neon Discord**: https://discord.gg/neondatabase
- **Connection Issues**: Check Neon status page

---

**Neon Setup Complete!** 🎉

Your PostgreSQL database is ready to use with Render deployment.
