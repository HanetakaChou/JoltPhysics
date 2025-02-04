// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <TestFramework.h>

#include <Utils/CustomMemoryHook.h>

#if defined(_DEBUG) && !defined(JPH_DISABLE_CUSTOM_ALLOCATOR) && !defined(JPH_COMPILER_MINGW)

// Global to turn checking on/off
static bool sEnableCustomMemoryHook = false;

// Local to temporarily disable checking
static thread_local int sDisableCustomMemoryHook = 1;

// Local to keep track if we're coming through the custom allocator
static thread_local bool sInCustomAllocator = false;

// Struct to put on the stack to flag that we're in the custom memory allocator
struct InCustomAllocator
{
	InCustomAllocator()
	{
		JPH_ASSERT(!sInCustomAllocator);
		sInCustomAllocator = true;
	}

	~InCustomAllocator()
	{
		JPH_ASSERT(sInCustomAllocator);
		sInCustomAllocator = false;
	}
};

// Add a tag to an allocation to track if it is aligned / unaligned
static void *TagAllocation(void *inPointer, size_t inAlignment, char inMode)
{
	if (inPointer == nullptr)
		return nullptr;

	uint8 *p = reinterpret_cast<uint8 *>(inPointer);
	*p = inMode;
	return p + inAlignment;
}

// Remove tag from allocation
static void *UntagAllocation(void *inPointer, size_t inAlignment, char inMode)
{
	if (inPointer == nullptr)
		return nullptr;

	uint8 *p = reinterpret_cast<uint8 *>(inPointer) - inAlignment;
	JPH_ASSERT(*p == inMode);
	*p = 0;
	return p;
}

static void *AllocateHook(size_t inSize)
{
	JPH_ASSERT(inSize > 0);
	InCustomAllocator ica;
	return TagAllocation(malloc(inSize + 16), 16, 'U');
}

static void *ReallocateHook(void *inBlock, size_t inOldSize, size_t inNewSize)
{
	JPH_ASSERT(inNewSize > 0);
	InCustomAllocator ica;
	return TagAllocation(realloc(UntagAllocation(inBlock, 16, 'U'), inNewSize + 16), 16, 'U');
}

static void FreeHook(void *inBlock)
{
	InCustomAllocator ica;
	free(UntagAllocation(inBlock, 16, 'U'));
}

static void *AlignedAllocateHook(size_t inSize, size_t inAlignment)
{
	JPH_ASSERT(inSize > 0 && inAlignment > 0 && inAlignment <= 64);
	InCustomAllocator ica;
	return TagAllocation(_aligned_malloc(inSize + 64, inAlignment), 64, 'A');
}

static void AlignedFreeHook(void *inBlock)
{
	InCustomAllocator ica;
	_aligned_free(UntagAllocation(inBlock, 64, 'A'));
}

static int MyAllocHook(int nAllocType, void *pvData, size_t nSize, int nBlockUse, long lRequest, const unsigned char * szFileName, int nLine) noexcept
{
	JPH_ASSERT(!sEnableCustomMemoryHook || sDisableCustomMemoryHook <= 0 || sInCustomAllocator);
	return true;
}

JPH_NAMESPACE_BEGIN

AllocateFunction Allocate = AllocateHook;
ReallocateFunction Reallocate = ReallocateHook;
FreeFunction Free = FreeHook;
AlignedAllocateFunction AlignedAllocate = AlignedAllocateHook;
AlignedFreeFunction AlignedFree = AlignedFreeHook;

JPH_NAMESPACE_END

void RegisterCustomMemoryHook()
{
	_CrtSetAllocHook(MyAllocHook);
}

void EnableCustomMemoryHook(bool inEnable)
{
	sEnableCustomMemoryHook = inEnable;
}

bool IsCustomMemoryHookEnabled()
{
	return sEnableCustomMemoryHook;
}

DisableCustomMemoryHook::DisableCustomMemoryHook()
{
	sDisableCustomMemoryHook--;
}

DisableCustomMemoryHook::~DisableCustomMemoryHook()
{
	sDisableCustomMemoryHook++;
}

#else

DisableCustomMemoryHook::DisableCustomMemoryHook()
{
}

DisableCustomMemoryHook::~DisableCustomMemoryHook()
{
}

static void *AllocateHook(size_t inSize)
{
	return malloc(inSize);
}

static void *ReallocateHook(void *inBlock, size_t inOldSize, size_t inNewSize)
{
	return realloc(inBlock, inNewSize);
}

static void FreeHook(void *inBlock)
{
	free(inBlock);
}

static void *AlignedAllocateHook(size_t inSize, size_t inAlignment)
{
#if defined(JPH_PLATFORM_WINDOWS)
	// Microsoft doesn't implement posix_memalign
	return _aligned_malloc(inSize, inAlignment);
#else
	void *block = nullptr;
	JPH_SUPPRESS_WARNING_PUSH
	JPH_GCC_SUPPRESS_WARNING("-Wunused-result")
	JPH_CLANG_SUPPRESS_WARNING("-Wunused-result")
	posix_memalign(&block, inAlignment, inSize);
	JPH_SUPPRESS_WARNING_POP
	return block;
#endif
}

static void AlignedFreeHook(void *inBlock)
{
#if defined(JPH_PLATFORM_WINDOWS)
	_aligned_free(inBlock);
#else
	free(inBlock);
#endif
}

JPH_NAMESPACE_BEGIN

AllocateFunction Allocate = AllocateHook;
ReallocateFunction Reallocate = ReallocateHook;
FreeFunction Free = FreeHook;
AlignedAllocateFunction AlignedAllocate = AlignedAllocateHook;
AlignedFreeFunction AlignedFree = AlignedFreeHook;

JPH_EXPORT void RegisterDefaultAllocator() {}

JPH_NAMESPACE_END

#endif // _DEBUG && !JPH_DISABLE_CUSTOM_ALLOCATOR && !JPH_COMPILER_MINGW
