# 502 Bad Gateway Fix Guide - Render Deployment

## Problem
Backend service showing `502 Bad Gateway` error - service not starting or crashing during startup.

## Root Causes

### Most Common Causes:
1. **Missing Environment Variables** (most likely)
   - `OPENAI_API_KEY` not set
   - `QDRANT_URL` not set
   - `QDRANT_API_KEY` not set
   - `DATABASE_URL` not set

2. **Database Connection Failed**
   - Invalid `DATABASE_URL`
   - Database not accessible from Render

3. **Qdrant Connection Failed**
   - Invalid `QDRANT_URL` or `QDRANT_API_KEY`
   - Network issues

4. **Application Startup Error**
   - Python import errors
   - Missing dependencies
   - Code syntax errors

---

## 🔍 Step 1: Check Render Logs

### How to Access Logs:
1. Go to: https://dashboard.render.com/
2. Click on your service: `rag-chatbot-api`
3. Click **Logs** tab (left sidebar)
4. Scroll down to see latest errors

### What to Look For:
- **"ValidationError"** → Missing environment variables
- **"Connection refused"** → Database/Qdrant connection issue
- **"ImportError"** → Missing Python package
- **"NameError"** or **"AttributeError"** → Code issue

### Common Error Messages:

#### Error 1: Missing Environment Variable
```
ValidationError: 1 validation error for Settings
openai_api_key
  Field required
```
**Fix**: Add `OPENAI_API_KEY` in Render Environment variables

#### Error 2: Database Connection Failed
```
OperationalError: could not connect to server
Connection refused
```
**Fix**: Check `DATABASE_URL` is correct and database is accessible

#### Error 3: Qdrant Connection Failed
```
ConnectionError: Failed to connect to Qdrant
```
**Fix**: Verify `QDRANT_URL` and `QDRANT_API_KEY`

---

## ✅ Step 2: Verify Environment Variables

Render Dashboard → Your Service → **Environment** tab

### Required Variables Checklist:

#### 1. OpenAI Configuration
```
OPENAI_API_KEY=sk-... (your OpenAI API key)
OPENAI_CHAT_MODEL=gpt-4o-mini
OPENAI_TEMPERATURE=0.3
OPENAI_MAX_TOKENS=1000
```

#### 2. Qdrant Configuration
```
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=book_content
QDRANT_VECTOR_SIZE=768
```

#### 3. Database Configuration
```
DATABASE_URL=postgresql://user:password@host:port/database?sslmode=require
```

#### 4. Application Configuration
```
ENVIRONMENT=production
CORS_ORIGINS=http://localhost:3000,https://muzaffar401.github.io,https://hamnakh.github.io
RATE_LIMIT_PER_MINUTE=10
LOG_LEVEL=INFO
OMP_NUM_THREADS=1
```

### How to Add/Update Variables:
1. Click **Environment** tab
2. Click **Add Environment Variable** button
3. Enter **Key** and **Value**
4. Click **Save Changes**
5. **Manual Deploy** → **"Deploy latest commit"** (service restart required)

---

## 🔧 Step 3: Test Database Connection

### Test Neon PostgreSQL Connection:
```bash
# Test from your local machine
psql "postgresql://neondb_owner:npg_8mHhd6OpUAjN@ep-curly-voice-a4yn2qyo-pooler.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require"
```

### Verify Tables Exist:
```sql
\dt
-- Should show: chat_sessions, chat_queries, chat_responses, feedback
```

### If Connection Fails:
1. Go to Neon Dashboard: https://console.neon.tech/
2. Regenerate connection string
3. Update `DATABASE_URL` in Render

---

## 🔧 Step 4: Test Qdrant Connection

### Verify Qdrant Credentials:
1. Go to Qdrant Cloud: https://cloud.qdrant.io/
2. Check your cluster URL and API key
3. Verify collection `book_content` exists

### Test Connection (Local):
```python
from qdrant_client import QdrantClient

client = QdrantClient(
    url="YOUR_QDRANT_URL",
    api_key="YOUR_QDRANT_API_KEY"
)

# Test connection
collections = client.get_collections()
print(collections)
```

---

## 🚀 Step 5: Manual Restart After Fix

### After Adding/Updating Environment Variables:

1. **Manual Deploy** dropdown (top right)
2. Select **"Deploy latest commit"**
3. Wait for deployment (~3-5 minutes)
4. Check logs for success message:
   ```
   INFO: Application startup complete.
   INFO: Uvicorn running on http://0.0.0.0:10000
   ```

---

## 🐛 Troubleshooting Specific Errors

### Issue 1: "Field required" Validation Error

**Problem**: Required environment variable missing

**Solution**:
1. Check Render logs for which field is missing
2. Add the missing variable in Environment tab
3. Restart service

### Issue 2: Database Connection Timeout

**Problem**: `DATABASE_URL` correct hai but connection timeout

**Possible Causes**:
- Database URL me `pooler` use nahi kar rahe
- Firewall blocking Render IPs

**Solution**:
1. Use **pooled connection string** from Neon:
   ```
   postgresql://user:pass@ep-xxx-xxx-pooler.region.aws.neon.tech/db?sslmode=require
   ```
2. Ensure `sslmode=require` is in URL

### Issue 3: Build Fails

**Problem**: Build command fails during deployment

**Check**:
- `requirements.txt` has all dependencies
- PyTorch installation successful (can be slow)

**Solution**:
- Check **Build Logs** tab in Render
- Look for specific package installation errors

### Issue 4: Service Starts But Immediately Crashes

**Problem**: Service starts then crashes after a few seconds

**Check Logs For**:
- Application errors after startup
- Memory issues (free tier has limits)
- Database migration errors

---

## 📋 Quick Diagnostic Checklist

Run through this checklist:

- [ ] All required environment variables are set in Render
- [ ] `DATABASE_URL` is correct and accessible
- [ ] `QDRANT_URL` and `QDRANT_API_KEY` are correct
- [ ] `OPENAI_API_KEY` is valid
- [ ] Service restarted after adding variables
- [ ] Render logs show "Application startup complete"
- [ ] Health check endpoint works: `https://chatbot-rag-krlg.onrender.com/`

---

## 🎯 Expected Result After Fix

✅ **Render Logs**: 
```
INFO: Application startup complete.
INFO: Uvicorn running on http://0.0.0.0:10000
```

✅ **Health Check**:
- URL: `https://chatbot-rag-krlg.onrender.com/`
- Response: `{"name":"RAG Chatbot API","version":"1.0.0",...}`

✅ **API Endpoints Working**:
- `GET /api/health` → 200 OK
- `POST /api/session` → 201 Created

---

## 📞 Still Getting 502?

If service still shows 502 after fixing environment variables:

1. **Share Render Logs** (last 100 lines)
2. **Share Environment Variables** (keys only, not values)
3. **Check Build Logs** for any build-time errors

**Common Final Checks**:
- Database migrations run? (`alembic upgrade head`)
- Qdrant collection exists and has data?
- OpenAI API key has sufficient credits?

---

**Fix Applied!** 🚀

After setting all required environment variables and restarting, the 502 error should be resolved.
