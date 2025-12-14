# Quick 502 Bad Gateway Fix

## Immediate Steps

### 1. Check Render Logs (MUST DO FIRST)
1. Go to: https://dashboard.render.com/
2. Click service: `rag-chatbot-api`
3. Click **Logs** tab
4. Scroll to bottom - latest errors dekho

**Common Errors You'll See:**
- ❌ `ValidationError: Field required` → Missing environment variable
- ❌ `Connection refused` → Database/Qdrant connection failed
- ❌ `ImportError` → Missing package

### 2. Verify ALL Required Environment Variables

Render Dashboard → Your Service → **Environment** tab

**Must Have These:**
```
✅ OPENAI_API_KEY
✅ QDRANT_URL
✅ QDRANT_API_KEY  
✅ DATABASE_URL
```

**Other Required:**
```
✅ OPENAI_CHAT_MODEL (default: gpt-4o-mini)
✅ QDRANT_COLLECTION_NAME (default: book_content)
✅ ENVIRONMENT (default: production)
```

### 3. After Adding Variables - RESTART

1. Click **Manual Deploy** (top right)
2. Select **"Deploy latest commit"**
3. Wait 3-5 minutes
4. Check logs again

### 4. Test Health Endpoint

After restart, test:
```
https://chatbot-rag-krlg.onrender.com/
```

Should return JSON, not 502 error.

---

## Most Likely Issue

**Missing `OPENAI_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`, or `DATABASE_URL`**

Render logs me exact error dikhega. Share logs if still stuck!
